#include <Arduino.h>
#include <PID_v1.h>

// ===== Pins =====
const uint8_t PIN_EDGE_A = A2;   // Comparator A -> D2 (master)
const uint8_t PIN_EDGE_B = A3;   // Comparator B -> D3 (slave)

const uint8_t MASTER_PWM = 5;   // Motor A PWM
const uint8_t MASTER_DIR = 9;   // Motor A DIR (set level you use for forward)
const uint8_t SLAVE_PWM  = 6;   // Motor B PWM
const uint8_t SLAVE_DIR  = 10;   // Motor B DIR (set level you use for forward)

// ===== Fixed settings =====
const uint8_t MASTER_PWM_BASE = 180;  // open-loop base for master (0..255)
const uint8_t SLAVE_PWM_BASE  = 180;  // nominal base for slave (0..255)
const uint8_t SLAVE_PWM_MIN   = 20;  // helps overcome static friction
const uint8_t SLAVE_PWM_MAX   = 255;

const uint16_t EDGE_LOCKOUT_US = 100;   // ignore edges that are too close
const uint32_t CTRL_DT_US      = 100;   // control loop period (µs) -- we'll run fast; PID runs at 1 ms

// ===== Desired mechanical phase offset =====
// positive = A leads B by this fraction of a cycle (0.25 = +90°, 0.5 = 180°)
volatile float phaseOffset_cycles = 0.5f;

// ===== Sensor zero-voltage parameters (mechanical zero) =====
// Voltage (0..3.3 V) at which each motor’s eccentric mass is at "downwards" position.
volatile float ZERO_A_V = 1.66f;   // volts for Motor A mechanical zero
volatile float ZERO_B_V = 1.59f;   // volts for Motor B mechanical zero
const float VSPAN = 3.3f;          // sensor full-scale volts

// ===== ISR state =====
volatile uint32_t tA = 0, pA = 0, perA = 0;
volatile uint32_t tB = 0, pB = 0, perB = 0;

void isrA() {
  uint32_t t = micros();
  if (t - tA < EDGE_LOCKOUT_US) return;
  pA = tA; tA = t;
  if (pA) perA = tA - pA;
}

void isrB() {
  uint32_t t = micros();
  if (t - tB < EDGE_LOCKOUT_US) return;
  pB = tB; tB = t;
  if (pB) perB = tB - pB;
}

// Wrap signed error into (-T/2, +T/2]
static inline int32_t wrapHalfPeriod(int32_t e_us, uint32_t T_us) {
  if (!T_us) return e_us;
  int32_t half = (int32_t)(T_us / 2u);
  while (e_us >  half) e_us -= (int32_t)T_us;
  while (e_us <= -half) e_us += (int32_t)T_us;
  return e_us;
}

// Wrap a normalized cycles value into (-0.5, +0.5]
static inline float wrapCycles(float cyc) {
  while (cyc >  0.5f) cyc -= 1.0f;
  while (cyc <= -0.5f) cyc += 1.0f;
  return cyc;
}

// ===== PID plumbing (normalized error -> PWM command) =====
// We regulate normalized error e_norm = ((TA - TB)/T - desired_edge_offset) wrapped to (-0.5, +0.5].
// PID Input = e_norm (dimensionless), Setpoint = 0.0, Output = slave PWM command [min..max].
double PID_Input = 0.0;
double PID_Output = SLAVE_PWM_BASE;
double PID_Setpoint = 0.0;

// Gains on normalized error (dimensionless):
// Kp: PWM counts per cycle of phase error
// Ki: PWM counts per cycle per second
// Kd: PWM counts per cycle times seconds
double Kp = 45.0;   // start here, then tune
double Ki = 5.0;   // integral strength (per second)
double Kd = 0.0;  // derivative strength

PID pid(&PID_Input, &PID_Output, &PID_Setpoint, Kp, Ki, Kd, DIRECT);

// ===== helpers to change parameters at runtime (optional) =====
inline void setPhaseOffsetCycles(float cyc) {
  noInterrupts();
  phaseOffset_cycles = wrapCycles(cyc);
  interrupts();
}
inline void setZeroVoltageA(float v) { noInterrupts(); ZERO_A_V = v; interrupts(); }
inline void setZeroVoltageB(float v) { noInterrupts(); ZERO_B_V = v; interrupts(); }

void setup() {
  // DIR levels for forward — keep the LOW/HIGH you used
  pinMode(MASTER_DIR, OUTPUT);
  pinMode(SLAVE_DIR,  OUTPUT);
  digitalWrite(MASTER_DIR, LOW);
  digitalWrite(SLAVE_DIR,  LOW);

  // PWM outputs
  pinMode(MASTER_PWM, OUTPUT);
  pinMode(SLAVE_PWM,  OUTPUT);
  analogWrite(MASTER_PWM, MASTER_PWM_BASE);
  analogWrite(SLAVE_PWM,  (uint8_t)PID_Output);  // start at base via PID_Output

  // Edge inputs (pull-ups for open-collector comparators)
  pinMode(PIN_EDGE_A, INPUT_PULLUP);
  pinMode(PIN_EDGE_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_EDGE_A), isrA, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_EDGE_B), isrB, FALLING);

  // === PID setup ===
  pid.SetOutputLimits(SLAVE_PWM_MIN, SLAVE_PWM_MAX);  // (min..max) PWM
  pid.SetSampleTime(1);                                // 1 ms internal update
  pid.SetMode(AUTOMATIC);                              // enable regulation
}

void loop() {
  static uint32_t tNext = micros();
  static int16_t lastCmd = SLAVE_PWM_BASE;

  if ((int32_t)(micros() - tNext) >= 0) {
    tNext += CTRL_DT_US;

    // Snapshot ISR & calibration vars
    noInterrupts();
    uint32_t TA = tA, TB = tB;
    uint32_t TA_per = perA, TB_per = perB;
    float off_cyc_user = phaseOffset_cycles;
    float za = ZERO_A_V, zb = ZERO_B_V;
    interrupts();

    // Choose period T for normalization: prefer master's
    uint32_t T = (TA_per ? TA_per : TB_per);

    // If we don't have valid edges yet: gently return toward base, skip PID compute
    if (T == 0 || TA == 0 || TB == 0) {
      // slew toward base
      int16_t base = SLAVE_PWM_BASE;
      const int16_t SLEW = 4;
      if (base > lastCmd + SLEW)      lastCmd += SLEW;
      else if (base < lastCmd - SLEW) lastCmd -= SLEW;
      else                            lastCmd  = base;
      if (lastCmd < SLAVE_PWM_MIN) lastCmd = SLAVE_PWM_MIN;
      if (lastCmd > SLAVE_PWM_MAX) lastCmd = SLAVE_PWM_MAX;
      analogWrite(SLAVE_PWM, (uint8_t)lastCmd);
      return;
    }

    // ---- Convert desired MECHANICAL phase to required EDGE offset (cycles) ----
    // edge_offset = mech_offset + (ZERO_A - ZERO_B)/VSPAN
    float calib_edge_offset_cyc = (za - zb) / VSPAN;
    float desired_edge_offset_cyc = wrapCycles(off_cyc_user + calib_edge_offset_cyc);

    // ---- Build normalized error e_norm in (-0.5..+0.5] ----
    // Time error (A leads positive), subtract desired offset (in time), then wrap:
    int32_t desired_offset_us = (int32_t)lrintf(desired_edge_offset_cyc * (float)T);
    int32_t e_us = (int32_t)(TA - TB) - desired_offset_us;
    e_us = wrapHalfPeriod(e_us, T);
    float e_norm = (float)e_us / (float)T;  // dimensionless phase error

    // ---- Feed PID ----
    PID_Setpoint = 0.0;         // want zero normalized error
    PID_Input    = (double)e_norm;

    // Let the PID compute at its own 1 ms internal pace (uses millis()).
    if (pid.Compute()) {
      // Optional output slew limiting to avoid abrupt steps
      const int16_t SLEW = 4;
      int16_t target = (int16_t)PID_Output;
      if (target > lastCmd + SLEW)      lastCmd += SLEW;
      else if (target < lastCmd - SLEW) lastCmd -= SLEW;
      else                              lastCmd  = target;

      // Safety clamp (already limited by PID, this is just belt & suspenders)
      if (lastCmd < SLAVE_PWM_MIN) lastCmd = SLAVE_PWM_MIN;
      if (lastCmd > SLAVE_PWM_MAX) lastCmd = SLAVE_PWM_MAX;

      analogWrite(SLAVE_PWM, (uint8_t)lastCmd);
    }
  }
}
