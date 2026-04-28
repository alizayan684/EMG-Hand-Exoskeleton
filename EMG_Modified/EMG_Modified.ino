// ============================================================
// ESP32‑S3 Combined – EMG inference + Impedance Motor Control
// Uses pre-recorded test data to simulate real‑time EMG
// ============================================================

#include <AccelStepper.h>
#include "dual_model.h"
#include "test_emg_data.h"

// ---------- Motor & Control Constants (unchanged) ----------
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
const float MAX_CABLE_CM  = 10.0;           // full pull distance

const float Kd = 20.0;     // stiffness N/cm
const float Bd = 10.0;     // damping  N·s/cm
const float M  = 1.0;      // virtual mass kg

// ---------- EMG Model Constants ----------
// Scaling values from the Colab Scaler cell (copy from EMG_Modified.ino)
float scaler_mean[] = {0.05661, 0.09641, 0.07488, 16.1463, 24.2009, 19.9889,
                       0.07299, 0.12500, 0.09832, 97.9235, 72.5543, 101.1743,
                       174.7765, 165.2671, 180.1056};
float scaler_std[]  = {0.06671, 0.09636, 0.10294, 15.3356, 23.9686, 22.1113,
                       0.08422, 0.12751, 0.13431, 27.8236, 26.6055, 26.5265,
                       28.4202, 32.0256, 32.7791};

const char* GESTURES[] = {"Fist", "Thumb", "Flex", "Ext", "Rest"}; // index 0 = Fist

// Because the test data uses 375 samples per window at (presumably) 1000 Hz,
// each window lasts 0.375 s. We update predictions every ~400 ms to match.
const unsigned long WINDOW_DURATION_MS = 400;

// ---------- Global Prediction Variables ----------
// These replace the old placeholder values. They are updated by the
// feature‑extraction + inference block at every window transition.
float intensity           = 0.0;   // 0.0 .. 1.0 (predicted contraction intensity)
bool  contraction_detected = false; // true only when gesture == Fist

// ---------- Motor Object ----------
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// Motor control state
float Xd_cm = 0.0;
unsigned long prev_control_time;
long prev_steps = 0;

// Window cycling state
int current_window = 0;
unsigned long last_window_time = 0;

// ---------- Feature Extraction (from EMG_Modified.ino) ----------
void extractFeatures(int windowIdx, float *features) {
  // features must be an array of length 15
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

    // Store computed features
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

  Serial.println("Combined sketch running – EMG windows control the motor continuously.");
}

// ---------- Main Loop ----------
void loop() {
  // --- Update prediction every WINDOW_DURATION_MS ---
  if (millis() - last_window_time >= WINDOW_DURATION_MS) {
    last_window_time = millis();

    // Cycle to the next test window
    current_window = (current_window + 1) % NUM_TEST_WINDOWS;

    // 1. Extract features for this window
    float features[15];
    extractFeatures(current_window, features);

    // 2. Scale features
    float scaled[15];
    for (int i = 0; i < 15; i++) {
      scaled[i] = (features[i] - scaler_mean[i]) / scaler_std[i];
    }

    // 3. Run model inference
    int gesture_idx = predict_gesture(scaled);
    float pred_intensity = predict_intensity(scaled);

    // 4. Update global motor setpoints
    intensity             = pred_intensity;            // 0.0 .. 1.0
    contraction_detected  = (gesture_idx == 0);        // true only for Fist

    // Debug info for this window
    Serial.print("Window "); Serial.print(current_window);
    Serial.print(" | Label: "); Serial.print(GESTURES[test_labels[current_window]]);
    Serial.print(" | Pred: "); Serial.print(GESTURES[gesture_idx]);
    Serial.print(" | Intensity: "); Serial.println(intensity * 100, 1);
  }

  // --- Motor Control (runs every loop iteration) ---
  unsigned long now = millis();
  float dt = (now - prev_control_time) / 1000.0f;
  if (dt <= 0.0) dt = 0.2f;              // first loop safeguard

  // Compute desired position Xd from updated globals
  if (contraction_detected) {
    float A = AMAX * pow(1.0f - intensity, ALPHA);
    Xd_cm = A * MAX_CABLE_CM;
  } else {
    Xd_cm = 0.0f;                         // no assistance if not Fist
  }

  // Impedance controller (unchanged)
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

  // Feed the stepper and maintain ~100 Hz loop
  stepper.runSpeed();
  delay(10);

  // Optional verbose output (can be commented out for speed)
  // Serial.print("Xd:"); Serial.print(Xd_cm);
  // Serial.print(" X:"); Serial.print(X_cm);
  // Serial.print(" V:"); Serial.print(desired_velocity);
  // Serial.println();
}