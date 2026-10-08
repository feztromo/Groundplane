#ifndef FAN_CONTROL_HPP
#define FAN_CONTROL_HPP

#include <Arduino.h>
#include "Sensorvector.hpp"

// Settings written by the web UI, read by updateFans()
struct FanParams {
  volatile bool  enabled = false;  // master switch, always boots OFF
  volatile float forward = 0.0f;   // 0..1, drive strength along the line
  volatile float kp = 1.0f;
  volatile float ki = 0.0f;
  volatile float kd = 0.0f;
};
extern FanParams fanParams;

constexpr float LINE_REF = PI / 2.0f;  // forward direction along the line, sensor frame

struct FanState {
  float angle;      // filtered deviation angle (rad)
  float length;     // filtered deviation length, 0..1
  float lineAngle;  // filtered line orientation relative to LINE_REF (rad)
  float cmdAngle;   // combined thrust vector angle (rad)
  float cmdLength;  // combined thrust vector length, 0..1
  float thrust[3];  // per-fan thrust 0..1
};

// Attaches the ESCs and holds minimum throttle so they can arm. Blocks ~2 s.
void initFans();

// Filters the sensor vector, runs the PID, combines it with the forward drive,
// mixes the fans and writes the ESC pulses. Call once per loop.
FanState updateFans(const VectorResult& lineVector);

// Last result of updateFans(), for the web UI
FanState getFanState();

#endif // FAN_CONTROL_HPP