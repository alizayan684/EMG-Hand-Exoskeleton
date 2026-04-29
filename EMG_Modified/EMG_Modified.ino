// ============================================================
// ESP32‑S3 Combined – EMG inference + Impedance Motor Control
// With 3‑window sliding filters for classification & intensity
// ============================================================

#include <AccelStepper.h>
#include "dual_model.h"
#include "test_emg_data.h"

// ---------- Motor & Control Constants ----------
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

const long  STEPS_PER_REV    = 4096;
const float SHAFT_RADIUS_CM  = 0.5;
const float CIRCUMFERENCE    = 2 * PI * SHAFT_RADIUS_CM;      // cm/rev
const float STEPS_PER_CM     = STEPS_PER_REV / CIRCUMFERENCE; // steps/cm

const float AMAX          = 1.0;
const float ALPHA         = 1.0;
const float MAX_CABLE_CM  = 10.0;   // full pull distance (cm)

const float Kd = 20.0;   // stiffness N/cm
const float Bd = 10.0;   // damping  N·s/cm
const float M  = 1.0;    // virtual mass kg

// ---------- EMG Model Constants ----------
float scaler_mean[] = {0.05661, 0.09641, 0.07488, 16.1463, 24.2009, 19.9889,
                       0.07299, 0.12500, 0.09832, 97.9235, 72.5543, 101.1743,
                       174.7765, 165.2671, 180.1056};
float scaler_std[]  = {0.06671, 0.09636, 0.10294, 15.3356, 23.9686, 22.1113,
                       0.08422, 0.12751, 0.13431, 27.8236, 26.6055, 26.5265,
                       28.4202, 32.0256, 32.7791};

const char* GESTURES[] = {"Fist", "Thumb", "Flex", "Ext", "Rest"};
const int FIST_ID = 0;            // gesture index for "Fist"

const unsigned long WINDOW_DURATION_MS = 400;  // matches the 375‑sample window

// ---------- Sliding window buffers ----------
const int WINDOW_SIZE = 3;
int gesture_buffer[WINDOW_SIZE];   // stores last 3 predicted gesture indices
float intensity_buffer[WINDOW_SIZE]; // stores last 3 predicted intensities
int buffer_count = 0;              // how many predictions have been collected so far

// The filtered, final values used by the motor controller
bool contraction_detected = false; // true only when majority says Fist
float intensity = 0.0;             // 0.0 (rest) .. 1.0 (max contraction)

// ---------- Motor object ----------
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// Motor control state
float Xd_cm = 0.0;
unsigned long prev_control_time;
long prev_steps = 0;

// Window cycling state (for test data)
int current_window = 0;
unsigned long last_window_time = 0;

// ---------- Simple median of three floats ----------
float medianOfThree(float a, float b, float c) {
  // bubble sort for three values
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ---------- Feature extraction (identical to original) ----------
void extractFeatures(int windowIdx, float *features) {
  for (int c = 0; c < 3; c++) {
    float mav = 0, wl = 0, rms = 0;
    int zc = 0, ssc = 0;

    for (int i = 0; i < 375; i++) {
      float val = test_emg_data[windowIdx][c][i];
      mav += abs(val);
      rms += val * val;

      if (i > 0) {
        float prev = test_emg_data[windowIdx][c][i-1];
        wl += abs(val - prev);
        if (prev * val < 0 && abs(val - prev) > 0.01) zc++;

        if (i > 1) {
          float prev2 = test_emg_data[windowIdx][c][i-2];
          if ((prev - prev2) * (val - prev) < 0 && abs(val - prev) > 0.01) ssc++;
        }
      }
    }

    features[c]      = mav / 375.0;
    features[c + 3]  = wl;
    features[c + 6]  = sqrt(rms / 375.0);
    features[c + 9]  = (float)zc;
    features[c + 12] = (float)ssc;
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // Motor setup
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(0);
  stepper.setCurrentPosition(0);
  prev_steps = 0;
  prev_control_time = millis();

  // Initialize window timer
  last_window_time = millis();

  Serial.println("Combined sketch with sliding window filters – running");
}

// ---------- Main Loop ----------
void loop() {
  // --- 1. Periodic EMG prediction (every WINDOW_DURATION_MS) ---
  if (millis() - last_window_time >= WINDOW_DURATION_MS) {
    last_window_time = millis();

    // Cycle to the next test window
    current_window = (current_window + 1) % NUM_TEST_WINDOWS;

    // Extract features, scale, and infer
    float features[15];
    extractFeatures(current_window, features);

    float scaled[15];
    for (int i = 0; i < 15; i++) {
      scaled[i] = (features[i] - scaler_mean[i]) / scaler_std[i];
    }

    int raw_gesture = predict_gesture(scaled);
    float raw_intensity = predict_intensity(scaled);

    // --- 2. Update sliding window buffers ---
    // Shift existing values to the left
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
      gesture_buffer[i] = gesture_buffer[i + 1];
      intensity_buffer[i] = intensity_buffer[i + 1];
    }
    // Insert newest prediction at the end
    gesture_buffer[WINDOW_SIZE - 1] = raw_gesture;
    intensity_buffer[WINDOW_SIZE - 1] = raw_intensity;

    // Keep track of how many predictions we have (up to WINDOW_SIZE)
    if (buffer_count < WINDOW_SIZE) buffer_count++;

    // --- 3. Apply filtering once the buffer is full ---
    if (buffer_count >= WINDOW_SIZE) {
      // a) Majority vote for classification
      int fist_votes = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        if (gesture_buffer[i] == FIST_ID) fist_votes++;
      }

      // Decision logic:
      // - If at least 2 of 3 are "Fist"         -> contraction_detected = true
      // - If at least 2 of 3 are NOT "Fist"     -> contraction_detected = false
      // - Otherwise (no majority)               -> keep previous state (no change)
      if (fist_votes >= 2) {
        contraction_detected = true;
      } else if ((WINDOW_SIZE - fist_votes) >= 2) {
        contraction_detected = false;
      }
      // else: no majority (e.g., three different gestures) → leave contraction_detected as is

      // b) Median filter for intensity
      intensity = medianOfThree(intensity_buffer[0],
                                intensity_buffer[1],
                                intensity_buffer[2]);
    }

    // Debug output for this window
    Serial.print("Wdw "); Serial.print(current_window);
    Serial.print(" | Raw: "); Serial.print(GESTURES[raw_gesture]);
    Serial.print(" "); Serial.print(raw_intensity * 100, 0); Serial.print("%");
    if (buffer_count >= WINDOW_SIZE) {
      Serial.print(" | Vote: "); Serial.print(fist_votes);
      Serial.print("/3 -> "); Serial.print(contraction_detected ? "FIST" : "REST");
      Serial.print(" | MedInt: "); Serial.print(intensity * 100, 0); Serial.print("%");
    } else {
      Serial.print(" | (filling buffer)");
    }
    Serial.println();
  }

  // --- 4. Motor impedance control (runs every loop) ---
  unsigned long now = millis();
  float dt = (now - prev_control_time) / 1000.0f;
  if (dt <= 0.0) dt = 0.2f;   // first-loop fallback

  // Compute desired position Xd using the FILTERED globals
  if (contraction_detected) {
    float A = AMAX * pow(1.0f - intensity, ALPHA);
    Xd_cm = A * MAX_CABLE_CM;
  } else {
    Xd_cm = 0.0f;
  }

  // Impedance controller
  long current_steps = stepper.currentPosition();
  float X_cm = current_steps / (float)STEPS_PER_CM;

  float Xdot_cm_s = 0.0;
  if (dt > 0.001) {
    long step_diff = current_steps - prev_steps;
    Xdot_cm_s = (step_diff / (float)STEPS_PER_CM) / dt;
  }

  float F = Kd * (Xd_cm - X_cm) - Bd * Xdot_cm_s;
  float acceleration = F / M;
  float desired_velocity = Xdot_cm_s + acceleration * dt;
  desired_velocity = constrain(desired_velocity, -15.0f, 15.0f);

  float target_speed_steps = desired_velocity * STEPS_PER_CM;
  stepper.setSpeed(target_speed_steps);

  // Store state for next cycle
  prev_steps = current_steps;
  prev_control_time = millis();

  // Run the motor and keep ~100 Hz loop
  stepper.runSpeed();
  delay(10);
}
