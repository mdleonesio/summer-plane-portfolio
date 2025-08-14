// /snippets/state_machine.cpp
// Minimal autopilot state machine skeleton (Arduino/C++)
// Focus: readability, override logic, timed ramps, simple PD pitch hold.

#include <Arduino.h>

enum State { IDLE, THROTTLE_RAMP, ELEVATOR_RAMP, CLIMB, CRUISE };

struct RC {
  bool autoMode;         // true when AUTOPILOT switch is ON
  int  thr_us;           // manual throttle microseconds (1000–2000)
  int  ele_us;           // manual elevator microseconds (1000–2000)
  int  knobA;            // tuning knob (0–1023) e.g., ramp time / gain
  int  knobB;            // tuning knob (0–1023) e.g., pitch target
};

static State state = IDLE;
static uint32_t t_state_ms = 0;

static float pitch_deg = 0.0f, q_dps = 0.0f;   // IMU pitch and pitch rate
static int   thr_cmd = 1000;                   // microseconds to ESC
static int   ele_cmd = 1500;                   // elevator command (neutral = 1500)

const int THR_MIN = 1000, THR_ARM = 1300, THR_MAX = 2000;
const int SERVO_MIN = 1000, SERVO_MID = 1500, SERVO_MAX = 2000;

// Tunables (can be mapped from knobs)
float ramp_s     = 2.0f;   // throttle ramp duration (s)
float pitch_cmd  = 6.0f;   // target pitch (deg) for climb trigger
float Kp = 22.0f, Kd = 2.0f; // PD gains for pitch hold

// ---- Hardware I/O stubs (replace with your real functions) -----------------
RC    readRC();                      // read switches/knobs/sticks
void  readIMU(float& pitch_deg, float& q_dps); // update attitude + pitch rate
void  writeOutputs(int thr_us, int ele_us);    // send PWM to ESC/servo
void  logSample(State s);            // print CSV or write to SD
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // init PWM, IMU, RC, SD here…
}

void enter(State s) { state = s; t_state_ms = millis(); }

void loop() {
  static uint32_t t_prev = millis();
  uint32_t t_now = millis();
  float dt = (t_now - t_prev) * 0.001f; t_prev = t_now;

  RC rc = readRC();
  readIMU(pitch_deg, q_dps);

  // Map knobs to tunables (example)
  ramp_s    = 0.5f + 4.0f * (rc.knobA / 1023.0f);
  pitch_cmd = 2.0f + 10.0f * (rc.knobB / 1023.0f);

  // Manual override & arming safety
  if (!rc.autoMode) { enter(IDLE); writeOutputs(rc.thr_us, rc.ele_us); logSample(state); return; }

  switch (state) {
    case IDLE: {
      thr_cmd = THR_MIN; ele_cmd = SERVO_MID;
      if (rc.autoMode) enter(THROTTLE_RAMP);
    } break;

    case THROTTLE_RAMP: {
      float u = min(1.0f, (millis() - t_state_ms) / (1000.0f * ramp_s));
      thr_cmd = THR_MIN + int(u * (THR_MAX - THR_MIN) * 0.6f); // ramp to ~60% as example
      if (thr_cmd >= THR_ARM + 50) enter(ELEVATOR_RAMP);
    } break;

    case ELEVATOR_RAMP: {
      // bias elevator toward positive pitch; simple open-loop ramp toward target
      float u = min(1.0f, (millis() - t_state_ms) / 800.0f);   // ~0.8 s ramp
      int bias = int(120 * u);                                 // ~±120 us max bias example
      ele_cmd = constrain(SERVO_MID + bias, SERVO_MIN, SERVO_MAX);
      if (pitch_deg >= pitch_cmd) enter(CLIMB);
    } break;

    case CLIMB: {
      // PD pitch hold around pitch_cmd
      float e = (pitch_cmd - pitch_deg);
      float u = Kp * e - Kd * q_dps;
      ele_cmd = constrain(int(SERVO_MID + u), SERVO_MIN, SERVO_MAX);
      // After a few seconds, transition to cruise
      if (millis() - t_state_ms > 4000) enter(CRUISE);
    } break;

    case CRUISE: {
      // Hold a gentler pitch target (half of climb target)
      float e = (0.5f * pitch_cmd - pitch_deg);
      float u = 0.6f * Kp * e - Kd * q_dps;
      ele_cmd = constrain(int(SERVO_MID + u), SERVO_MIN, SERVO_MAX);
    } break;
  }

  // Output & logging
  thr_cmd = constrain(thr_cmd, THR_MIN, THR_MAX);
  writeOutputs(thr_cmd, ele_cmd);
  logSample(state);
}

// ---------------- Example stub implementations ------------------------------
RC readRC() {
  RC r; r.autoMode = true; r.thr_us = 1200; r.ele_us = SERVO_MID; r.knobA = 700; r.knobB = 500;
  return r; // Replace with your iBus/PWM reads
}
void readIMU(float& pitch, float& q) { /* replace with real IMU */ pitch += 0.0f; q = 0.0f; }
void writeOutputs(int thr_us, int ele_us) { /* replace with Servo.writeMicroseconds */ }
void logSample(State s) {
  // Replace with SD writes; CSV example:
  // Serial.printf("%lu,%d,%d,%.2f,%.2f,%d\n", millis(), thr_cmd, ele_cmd, pitch_deg, q_dps, (int)s);
}
