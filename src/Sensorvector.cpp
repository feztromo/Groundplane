#include "Sensorvector.hpp"

#define TWO_PI_F 6.28318530718f

// --- Sensor reading (mux control) ---

void initSensorArray() {
  pinMode(SENSOR_BIT_1, OUTPUT);
  pinMode(SENSOR_BIT_2, OUTPUT);
  pinMode(SENSOR_BIT_3, OUTPUT);
  pinMode(SENSOR_BIT_4, OUTPUT);
}

int readSensor(uint8_t channel) {
  digitalWrite(SENSOR_BIT_1, channel & 0x01);
  digitalWrite(SENSOR_BIT_2, (channel >> 1) & 0x01);
  digitalWrite(SENSOR_BIT_3, (channel >> 2) & 0x01);
  digitalWrite(SENSOR_BIT_4, (channel >> 3) & 0x01);

  delayMicroseconds(MUX_SETTLE_US);

  analogRead(SENSOR_ARRAY);  // throwaway read, clears charge from the previous channel

  int sum = 0;
  for (int k = 0; k < ADC_SAMPLES; k++) sum += analogRead(SENSOR_ARRAY);
  return sum / ADC_SAMPLES;
}

void readArray(int values[N_SENSORS]) {
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    values[i] = readSensor(i);
  }
}

// --- Vector computation ---

static constexpr float sensorAngles[N_SENSORS] = {
  0*TWO_PI_F/N_SENSORS,  1*TWO_PI_F/N_SENSORS,  2*TWO_PI_F/N_SENSORS,  3*TWO_PI_F/N_SENSORS,
  4*TWO_PI_F/N_SENSORS,  5*TWO_PI_F/N_SENSORS,  6*TWO_PI_F/N_SENSORS,  7*TWO_PI_F/N_SENSORS,
  8*TWO_PI_F/N_SENSORS,  9*TWO_PI_F/N_SENSORS,  10*TWO_PI_F/N_SENSORS, 11*TWO_PI_F/N_SENSORS,
  12*TWO_PI_F/N_SENSORS, 13*TWO_PI_F/N_SENSORS, 14*TWO_PI_F/N_SENSORS, 15*TWO_PI_F/N_SENSORS
};

static inline int wrapIdx(int i) { return (i % N_SENSORS + N_SENSORS) % N_SENSORS; }

static inline int ringDist(int a, int b) {
  int d = abs(a - b);
  return min(d, N_SENSORS - d);
}

VectorResult computeMidpointVector(const float values[N_SENSORS]) {
  // --- 1. Per-sensor low pass filter (temporal averaging) ---
  static float smooth[N_SENSORS];
  static bool initialized = false;

  if (!initialized) {
    for (int i = 0; i < N_SENSORS; i++) smooth[i] = values[i];
    initialized = true;
  } else {
    for (int i = 0; i < N_SENSORS; i++) smooth[i] += SENSOR_ALPHA * (values[i] - smooth[i]);
  }

  // --- 2. Neighbourhood average score (3 sensors) ---
  // A lone noisy channel is diluted, a real lobe (several bright neighbours) wins.
  float score[N_SENSORS];
  float mean = 0.0f;
  for (int i = 0; i < N_SENSORS; i++) {
    score[i] = (smooth[wrapIdx(i - 1)] + smooth[i] + smooth[wrapIdx(i + 1)]) / 3.0f;
    mean += smooth[i];
  }
  mean /= N_SENSORS;

  // --- 3. Peak 1: highest average ---
  int idx1 = 0;
  for (int i = 1; i < N_SENSORS; i++) {
    if (score[i] > score[idx1]) idx1 = i;
  }

  VectorResult result{0.0f, 0.0f, false};
  if (score[idx1] < mean + MIN_CONTRAST) return result;  // no real signal

  // --- 4. Peak 2: highest average at least MIN_SEP steps away from peak 1 ---
  int idx2 = -1;
  for (int i = 0; i < N_SENSORS; i++) {
    if (ringDist(i, idx1) < MIN_SEP) continue;
    if (idx2 < 0 || score[i] > score[idx2]) idx2 = i;
  }
  if (idx2 < 0) return result;

  // --- 5. Parabolic refinement on the smoothed values ---
  float refinedAngles[2];
  const int peakIdx[2] = { idx1, idx2 };
  for (int p = 0; p < 2; p++) {
    const int i = peakIdx[p];
    const float left   = smooth[wrapIdx(i - 1)];
    const float center = smooth[i];
    const float right  = smooth[wrapIdx(i + 1)];

    const float denom = 2.0f * (left - 2.0f * center + right);
    float offset = (denom != 0.0f) ? (left - right) / denom : 0.0f;
    offset = constrain(offset, -0.5f, 0.5f);

    refinedAngles[p] = sensorAngles[i] + offset * (TWO_PI_F / N_SENSORS);
  }

  // --- 6. Averaged vector ---
  const float x = (cosf(refinedAngles[0]) + cosf(refinedAngles[1])) * 0.5f;
  const float y = (sinf(refinedAngles[0]) + sinf(refinedAngles[1])) * 0.5f;

  result.length = sqrtf(x * x + y * y);
  result.angle  = atan2f(y, x);
  result.valid  = true;

  // Line orientation: direction of the chord between the two crossing points.
  // Unlike the midpoint angle, it does not flip 180 deg when the ring crosses the line.
  const float cx = cosf(refinedAngles[1]) - cosf(refinedAngles[0]);
  const float cy = sinf(refinedAngles[1]) - sinf(refinedAngles[0]);
  float lineAngle = atan2f(cy, cx);
  if (lineAngle >  PI / 2.0f) lineAngle -= PI;
  if (lineAngle <= -PI / 2.0f) lineAngle += PI;
  result.lineAngle = lineAngle;

  return result;
}