#ifndef SENSOR_VECTOR_HPP
#define SENSOR_VECTOR_HPP

#include <Arduino.h>
#include <math.h>

#define N_SENSORS 16

#define SENSOR_BIT_1 1
#define SENSOR_BIT_2 2
#define SENSOR_BIT_3 3
#define SENSOR_BIT_4 4
#define SENSOR_ARRAY 5

// ---- Tuning ----
constexpr int   ADC_SAMPLES    = 4;      // averaged reads per channel
constexpr int   MUX_SETTLE_US  = 10;     // settle time after switching channel
constexpr float SENSOR_ALPHA   = 0.4f;   // per-sensor low pass, 1 = off, smaller = smoother
constexpr int   MIN_SEP        = 3;      // min ring distance between the two peaks
constexpr float MIN_CONTRAST   = 150.0f; // ADC counts: strongest peak must exceed array mean by this

struct VectorResult {
  float angle;      // radians, midpoint angle (deviation direction, flips 180 deg across the line)
  float length;     // 0-1, cos(half the angular separation)
  bool  valid;
  float lineAngle;  // radians, orientation of the line itself, range (-pi/2, pi/2]
};

// Sets up the mux address pins. Call once from setup()
void initSensorArray();

// Selects mux channel (0-15) and returns an averaged analog value
int readSensor(uint8_t channel);

// Reads all 16 mux channels into values[0..15]
void readArray(int values[N_SENSORS]);

// Computes the midpoint direction/length vector from a 16-element sensor array.
// Call once per loop: keeps internal per-sensor filter state.
VectorResult computeMidpointVector(const float values[N_SENSORS]);

#endif // SENSOR_VECTOR_HPP