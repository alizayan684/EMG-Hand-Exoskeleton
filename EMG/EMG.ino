#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
// #include "tensorflow/lite/version.h"
#include "emg_model.h"
#include <AccelStepper.h>

// -------------------- EMG & Model Constants (from your code) --------------------
const int NUM_CHANNELS = 3;
const int WINDOW_SIZE = 200;
const int NUM_FEATURES = 15;
const float SESSION_MAX = 0.197055;

float scaler_mean[] = {0.080339, 0.063377, 0.133859, 12.7115, 10.1651, 24.3582,
                       0.103165, 0.080689, 0.171793, 40.3152, 54.6767, 59.0185,
                       104.8278, 100.0354, 100.4950};
float scaler_std[]  = {0.064822, 0.054830, 0.101812, 10.1716, 7.6505, 18.7954,
                       0.086308, 0.070877, 0.132965, 17.8953, 23.5268, 16.4412,
                       15.4987, 16.6755, 15.5819};

const char* GESTURES[] = {"Fist", "Thumb Flexion", "Mid+Ring Flexion", "Extension", "Rest"};

// TFLite globals
tflite::AllOpsResolver resolver;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
constexpr int kTensorArenaSize = 10 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

float window_buffer[NUM_CHANNELS][WINDOW_SIZE];

// -------------------- Motor & Control Constants --------------------
// Motor drive pins (adjust to your wiring)
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

// 28BYJ-48: 4096 steps per output shaft revolution (half‑step mode)
const long STEPS_PER_REV = 4096;
const float SHAFT_RADIUS_CM = 0.5;           // radius of the pulley / shaft
const float CIRCUMFERENCE = 2 * PI * SHAFT_RADIUS_CM;   // cm per revolution
const float STEPS_PER_CM = STEPS_PER_REV / CIRCUMFERENCE;

// Assistance mapping
const float AMAX = 1.0;
const float ALPHA = 1.0;
const float MAX_CABLE_CM = 10.0;              // full assistance = 10 cm pull

// Impedance & virtual mass constants (tune these later)
const float Kd = 20.0;    // stiffness N/m (or N/cm – we use cm here)
const float Bd = 10.0;    // damping N·s/m (or N·s/cm)
const float M  = 1.0;     // virtual mass kg

// Gesture confidence threshold
const float CONFIDENCE_THRESHOLD = 0.8;

// Motor object (HALF4WIRE gives smooth 8‑step sequence)
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// Control state
float Xd_cm = 0.0;                // desired cable position
unsigned long prev_control_time;
long prev_steps = 0;

void setup() {
  Serial.begin(115200);

  // --- TFLite setup (unchanged from your code) ---
  model = tflite::GetModel(g_emg_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }
  static tflite::MicroInterpreter static_interpreter(model, resolver,
                                                     tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);

  // --- Motor setup ---
  stepper.setMaxSpeed(2000);        // steps/s (hardware limit)
  stepper.setSpeed(0);              // start still
  stepper.setCurrentPosition(0);    // zero position = 0 cm
  prev_steps = 0;
  prev_control_time = millis();
}

void loop() {
  // --- Update control timing (dt = time since last motor command) ---
  unsigned long now = millis();
  float dt = (now - prev_control_time) / 1000.0;   // seconds
  if (dt <= 0.0) dt = 0.2;                         // fallback for first call

  // ===== 1. Collect 200 samples WHILE running the motor =====
  for (int i = 0; i < WINDOW_SIZE; i++) {
    window_buffer[0][i] = analogRead(34) * (3.3 / 4095.0);
    window_buffer[1][i] = analogRead(35) * (3.3 / 4095.0);
    window_buffer[2][i] = analogRead(36) * (3.3 / 4095.0);
    stepper.runSpeed();                          // keep motor stepping
    delayMicroseconds(1000);
  }

  // ===== 2. Feature extraction (unchanged) =====
  float features[NUM_FEATURES];
  float current_mavs[3];

  for (int c = 0; c < NUM_CHANNELS; c++) {
    float mav = 0, wl = 0, rms = 0;
    int zc = 0, ssc = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float val = window_buffer[c][i];
      mav += abs(val);
      rms += val * val;
      if (i > 0) {
        wl += abs(val - window_buffer[c][i - 1]);
        if (window_buffer[c][i - 1] * val < 0 && abs(val - window_buffer[c][i - 1]) > 0.01)
          zc++;
        if (i > 1) {
          float d1 = window_buffer[c][i - 1] - window_buffer[c][i - 2];
          float d2 = val - window_buffer[c][i - 1];
          if (d1 * d2 < 0 && abs(d1 - d2) > 0.01)
            ssc++;
        }
      }
    }
    mav /= WINDOW_SIZE;
    current_mavs[c] = mav;
    features[c]     = mav;
    features[c + 3] = wl;
    features[c + 6] = sqrt(rms / WINDOW_SIZE);
    features[c + 9] = (float)zc;
    features[c + 12]= (float)ssc;
  }

  // ===== 3. Scale and run inference =====
  for (int i = 0; i < NUM_FEATURES; i++) {
    input->data.f[i] = (features[i] - scaler_mean[i]) / scaler_std[i];
  }
  interpreter->Invoke();

  int best_gesture = 0;
  float max_score = 0;
  for (int i = 0; i < 5; i++) {
    if (output->data.f[i] > max_score) {
      max_score = output->data.f[i];
      best_gesture = i;
    }
  }

  // ===== 4. Intensity (I_patient / Imax) =====
  float fused_mav = (current_mavs[0] + current_mavs[1] + current_mavs[2]) / 3.0;
  float intensity = constrain(fused_mav / SESSION_MAX, 0.0, 1.0);

  // ===== 5. Decide if assistance is active =====
  bool contraction_detected = (best_gesture == 0) && (max_score > CONFIDENCE_THRESHOLD);

  // ===== 6. High‑level assistance (A(t) → Xd) =====
  if (contraction_detected) {
    float A = AMAX * pow(1.0 - intensity, ALPHA);
    Xd_cm = A * MAX_CABLE_CM;
  } else {
    Xd_cm = 0.0;    // no assistance → return to rest
  }

  // ===== 7. Low‑level impedance control =====
  // Position feedback from motor step counter
  long current_steps = stepper.currentPosition();
  float X_cm = current_steps / STEPS_PER_CM;

  // Estimate actual velocity
  float Xdot_cm_s = 0.0;
  if (dt > 0.001) {
    long step_diff = current_steps - prev_steps;
    Xdot_cm_s = (step_diff / STEPS_PER_CM) / dt;
  }

  // Force law (desired final velocity = 0)
  float F = Kd * (Xd_cm - X_cm) - Bd * Xdot_cm_s;

  // Virtual mass acceleration → integrate to desired speed
  float acceleration = F / M;
  float desired_velocity = Xdot_cm_s + acceleration * dt;

  // Clip to safe operation (prevent violent motion)
  desired_velocity = constrain(desired_velocity, -5.0, 5.0);  // cm/s

  // Convert to motor steps/s and command
  float target_speed_steps = desired_velocity * STEPS_PER_CM;
  stepper.setSpeed(target_speed_steps);

  // ===== 8. Store state for next cycle =====
  prev_steps = current_steps;
  prev_control_time = millis();

  // ===== 9. Debug output =====
  Serial.print("Gesture: "); Serial.print(GESTURES[best_gesture]);
  Serial.print(" | Conf: "); Serial.print(max_score);
  Serial.print(" | Intensity: "); Serial.print(intensity * 100);
  Serial.print(" % | Active: "); Serial.print(contraction_detected ? "YES" : "NO");
  Serial.print(" | Xd: "); Serial.print(Xd_cm);
  Serial.print(" cm | X: "); Serial.print(X_cm);
  Serial.print(" cm | V: "); Serial.print(desired_velocity);
  Serial.println(" cm/s");
}