// ============================================================
// ESP32‑S3 Motor Control Test – Placeholder EMG Output
// Simulates a permanent contraction (Fist) at 50% intensity
// ============================================================

#include <AccelStepper.h>

// -------------------- Motor & Control Constants --------------------
// Motor driver pins (ULN2003)
#define IN1  8
#define IN2  9
#define IN3 10
#define IN4 11

// 28BYJ-48 half‑step: 4096 steps per output‑shaft revolution
const long  STEPS_PER_REV    = 4096;
const float SHAFT_RADIUS_CM  = 0.5;
const float CIRCUMFERENCE    = 2 * PI * SHAFT_RADIUS_CM;  // cm / rev
const float STEPS_PER_CM     = STEPS_PER_REV / CIRCUMFERENCE;

// Assistance mapping
const float AMAX          = 1.0;
const float ALPHA         = 1.0;
const float MAX_CABLE_CM  = 10.0;            // full assistance = 10 cm pull

// Impedance constants (tune these to your mechanics)
const float Kd = 20.0;    // stiffness N/cm
const float Bd = 10.0;    // damping  N·s/cm
const float M  = 1.0;     // virtual mass kg

// -------------------------------------------------
// PLACEHOLDER OUTPUTS – replace these with your own test scenarios
// -------------------------------------------------
float intensity           = 0.5;   // 0.0 (rest) to 1.0 (max contraction)
bool contraction_detected = true;  // true = Fist gesture recognised

// Motor object (HALF4WIRE uses 8‑step sequence – smooth)
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// Control state
float Xd_cm = 0.0;
unsigned long prev_control_time;
long prev_steps = 0;

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // --- Motor Setup ---
  stepper.setMaxSpeed(2000);        // steps/s (hardware limit)
  stepper.setSpeed(0);
  stepper.setCurrentPosition(0);    // zero = 0 cm
  prev_steps = 0;
  prev_control_time = millis();

  Serial.println("Motor test running – simulated contraction @ 50% intensity");
}

void loop() {
  // --- Compute dt (time since last motor command) ---
  unsigned long now = millis();
  float dt = (now - prev_control_time) / 1000.0;   // seconds
  if (dt <= 0.0) dt = 0.2;                         // first‑loop fallback

  // ===== 1. High‑level desired position Xd =====
  // (No model – just use the placeholder values)
  if (contraction_detected) {
    // A(t) = AMAX * (1 - intensity)^alpha
    float A = AMAX * pow(1.0 - intensity, ALPHA);
    Xd_cm = A * MAX_CABLE_CM;
  } else {
    Xd_cm = 0.0;    // no assistance → return to rest
  }

  // ===== 2. Low‑level impedance control (unchanged) =====
  long current_steps = stepper.currentPosition();
  float X_cm = current_steps / STEPS_PER_CM;

  float Xdot_cm_s = 0.0;
  if (dt > 0.001) {
    long step_diff = current_steps - prev_steps;
    Xdot_cm_s = (step_diff / STEPS_PER_CM) / dt;
  }

  // Force law (desired acceleration = 0)
  float F = Kd * (Xd_cm - X_cm) - Bd * Xdot_cm_s;

  // Virtual mass integration
  float acceleration = F / M;
  float desired_velocity = Xdot_cm_s + acceleration * dt;

  // Clip to safe operation
  desired_velocity = constrain(desired_velocity, -5.0, 5.0);  // cm/s

  // Convert to motor steps/s and command
  float target_speed_steps = desired_velocity * STEPS_PER_CM;
  stepper.setSpeed(target_speed_steps);

  // ===== 3. Store state for next cycle =====
  prev_steps = current_steps;
  prev_control_time = millis();

  // ===== 4. Debug output =====
  Serial.print("Xd: "); Serial.print(Xd_cm);
  Serial.print(" cm | X: "); Serial.print(X_cm);
  Serial.print(" cm | V: "); Serial.print(desired_velocity);
  Serial.print(" cm/s | Steps/s: "); Serial.print(target_speed_steps);
  // Show the simulated inputs
  Serial.print(" | Simulated: intensity="); Serial.print(intensity * 100);
  Serial.print("%, contraction="); Serial.println(contraction_detected ? "YES" : "NO");

  // Optional: add a small loop delay so the motor can run smoothly
  // (without this, the loop runs as fast as possible)
  stepper.runSpeed();             // feed the motor
  delay(10);                      // ~100 Hz control loop
}