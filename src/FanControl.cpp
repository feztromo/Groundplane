#include "FanControl.hpp"
#include <Servo.h>
#include <ArduPID.h>
#include <math.h>

FanParams fanParams;

static Servo M1, M2, M3;

// ESC pins
constexpr int PIN_M1 = 6;
constexpr int PIN_M2 = 7;
constexpr int PIN_M3 = 8;

// Low pass coefficients: 0 < ALPHA <= 1 (smaller = smoother but slower)
constexpr float ALPHA_VECTOR = 0.1f;  // deviation vector
constexpr float ALPHA_LINE   = 0.1f;  // line orientation
constexpr float ALPHA_CONF   = 0.1f;  // "line is visible" confidence

constexpr float LENGTH_CUTOFF = 0.05f;  // below this combined length all fans are off

// PID settings (the vector length is 0..1, so these are in the same units)
constexpr float PID_DT_MS     = 10.0f;  // ArduPID updates at most this often, match your loop rate
constexpr float PID_OUT_LIMIT = 2.0f;   // max correction per axis
constexpr float PID_I_LIMIT   = 0.5f;   // max integral term (anti windup)
constexpr float PID_D_LIMIT   = 1.0f;   // max derivative term (limits the kick after a reset)

// Fan mounting angles (radians), same frame as lineVector.angle.
// If the result is mirrored, swap fan 2 and fan 3 (or negate the angles).
constexpr float FAN_ANGLE[3] = { 0.0f, 2.0f * PI / 3.0f, 4.0f * PI / 3.0f };

// ESC pulse settings (microseconds)
constexpr int PULSE_OFF   = 1000;
constexpr int PULSE_START = 1050;  // lowest pulse where the fan actually spins
constexpr int PULSE_MAX   = 2000;

// Filter state
static float filtX = 0.0f, filtY = 0.0f;
static bool  vecInit = false;

static float lineS = 0.0f, lineC = 1.0f;  // line orientation, doubled-angle form
static bool  lineInit = false;
static float conf = 0.0f;

// PID state: one ArduPID per axis of the deviation vector
static ArduPID pidX, pidY;
static float appliedKp = -1.0f, appliedKi = -1.0f, appliedKd = -1.0f;
static bool  pidRunning = false;

static FanState lastState{};

static void configPid(ArduPID& pid) {
  pid.setSetpoint(0.0f);  // target: zero deviation
  pid.setDirection(ArduPID::FORWARD);
  pid.setDtMs(PID_DT_MS);
  pid.setOutputLimits(-PID_OUT_LIMIT, PID_OUT_LIMIT);
  pid.setILimits(-PID_I_LIMIT, PID_I_LIMIT);
  pid.setDLimits(-PID_D_LIMIT, PID_D_LIMIT);
}

// angle in radians, length 0..1, out[i] in 0..1
// The strongest fan always gets 'length', the weakest gets 0.
static void mixFans(float angle, float length, float out[3]) {
  float c[3];
  float cMin = 1e9f, cMax = -1e9f;

  for (int i = 0; i < 3; i++) {
    c[i] = cosf(angle - FAN_ANGLE[i]);
    if (c[i] < cMin) cMin = c[i];
    if (c[i] > cMax) cMax = c[i];
  }

  const float spread = cMax - cMin;
  const float len = constrain(length, 0.0f, 1.0f);

  for (int i = 0; i < 3; i++) {
    out[i] = len * (c[i] - cMin) / spread;
  }
}

// 0..1 -> pulse width, skipping the dead zone at the bottom
static int toPulse(float t) {
  if (t <= 0.001f) return PULSE_OFF;
  return PULSE_START + (int)(t * (PULSE_MAX - PULSE_START));
}

void initFans() {
  M1.attach(PIN_M1);
  M2.attach(PIN_M2);
  M3.attach(PIN_M3);

  M1.writeMicroseconds(PULSE_OFF);
  M2.writeMicroseconds(PULSE_OFF);
  M3.writeMicroseconds(PULSE_OFF);

  configPid(pidX);
  configPid(pidY);

  delay(2000);
}

FanState updateFans(const VectorResult& v) {
  const bool  enabled = fanParams.enabled;
  const float forward = constrain((float)fanParams.forward, 0.0f, 1.0f);
  const float kp = fanParams.kp;
  const float ki = fanParams.ki;
  const float kd = fanParams.kd;

  // --- 1. Deviation vector, low pass filtered as x/y (continuous through the line) ---
  if (v.valid) {
    const float x = v.length * cosf(v.angle);
    const float y = v.length * sinf(v.angle);
    if (!vecInit) {
      filtX = x; filtY = y; vecInit = true;
    } else {
      filtX += ALPHA_VECTOR * (x - filtX);
      filtY += ALPHA_VECTOR * (y - filtY);
    }
  } else {
    filtX -= ALPHA_VECTOR * filtX;  // fade out
    filtY -= ALPHA_VECTOR * filtY;
  }

  // --- 2. Line orientation, filtered in doubled-angle form (180 deg periodic) ---
  if (v.valid) {
    const float s2 = sinf(2.0f * v.lineAngle);
    const float c2 = cosf(2.0f * v.lineAngle);
    if (!lineInit) {
      lineS = s2; lineC = c2; lineInit = true;
    } else {
      lineS += ALPHA_LINE * (s2 - lineS);
      lineC += ALPHA_LINE * (c2 - lineC);
    }
  }
  conf += ALPHA_CONF * ((v.valid ? 1.0f : 0.0f) - conf);
  const float lineAngle = lineInit ? 0.5f * atan2f(lineS, lineC) : 0.0f;

  // --- 3. PID (ArduPID) on the deviation vector, setpoint = zero vector ---
  float ux = 0.0f, uy = 0.0f;
  if (enabled) {
    if (!pidRunning) {          // just switched on: start from a clean state
      pidX.reset();
      pidY.reset();
      pidRunning = true;
    }
    if (kp != appliedKp || ki != appliedKi || kd != appliedKd) {
      pidX.setTunings(kp, ki, kd);
      pidY.setTunings(kp, ki, kd);
      appliedKp = kp; appliedKi = ki; appliedKd = kd;
    }
    // ArduPID error = setpoint - input, so feeding -dev gives error = +dev
    ux = pidX.compute(-filtX);
    uy = pidY.compute(-filtY);
  } else if (pidRunning) {
    pidX.reset();
    pidY.reset();
    pidRunning = false;
  }

  // --- 4. Forward drive along the line (scaled down when the line isn't seen) ---
  const float heading = LINE_REF + lineAngle;
  const float f = forward * conf;
  const float fx = f * cosf(heading);
  const float fy = f * sinf(heading);

  // --- 5. Combine and limit to length 1 ---
  float tx = fx + ux;
  float ty = fy + uy;
  float mag = sqrtf(tx * tx + ty * ty);
  if (mag > 1.0f) { tx /= mag; ty /= mag; mag = 1.0f; }
  const float cmdAngle = atan2f(ty, tx);

  // --- 6. Mix and output ---
  FanState state{};
  state.angle     = atan2f(filtY, filtX);
  state.length    = sqrtf(filtX * filtX + filtY * filtY);
  state.lineAngle = lineAngle;
  state.cmdAngle  = cmdAngle;
  state.cmdLength = mag;

  if (!enabled || mag < LENGTH_CUTOFF) {
    state.thrust[0] = state.thrust[1] = state.thrust[2] = 0.0f;
    M1.writeMicroseconds(PULSE_OFF);
    M2.writeMicroseconds(PULSE_OFF);
    M3.writeMicroseconds(PULSE_OFF);
  } else {
    mixFans(cmdAngle, mag, state.thrust);
    M1.writeMicroseconds(toPulse(state.thrust[1]));
    M2.writeMicroseconds(toPulse(state.thrust[2]));
    M3.writeMicroseconds(toPulse(state.thrust[0]));
  }

  lastState = state;
  return state;
}

FanState getFanState() {
  return lastState;
}