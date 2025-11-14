#include <Arduino.h>
#include <PID_v1.h>

/*
  ========== Multi-motor phase control (Arduino Zero / SAMD21) ==========
  - 6 motors total; PWM pins: 3,4,5,6,7,8 (index 0..5)
  - Edge/comparator inputs (FALLING): A5,A4,A2,A3,A0,A1 (index 0..5)
  - Motor on PWM pin 5 is the reference (index 2 by default)
  - 5 simultaneous regulations: each non-reference motor tracks its own
    phase target relative to the reference
  - Direction pins are omitted
  - Per-motor "mechanical zero" calibration in volts (0..3.3V)
  - Phase targets are in cycles (1.0 -> 360°, 0.5 -> 180°, etc.)
*/

static const uint8_t NUM_MOTORS = 6;

// ---------------- Pin maps ----------------
static const uint8_t PWM_PINS[NUM_MOTORS]  = {3, 4, 5, 6, 7, 8};             // idx 0..5
static const uint8_t EDGE_PINS[NUM_MOTORS] = {A5, A4, A2, A3, A0, A1};       // idx 0..5

// Reference motor is the one on PWM pin 5 (preferably)
static const uint8_t REF_PWM_PIN = 5;
static int8_t REF_IDX = -1;   // resolved at setup()

// ---------------- Fixed settings ----------------
static const uint8_t PWM_BASE[NUM_MOTORS] = {
  150, 150, 150, 150, 150, 150  // reference can have a slightly different base if desired
};
static const uint8_t PWM_MIN[NUM_MOTORS]  = { 20, 20, 20, 20, 20, 20 };
static const uint8_t PWM_MAX[NUM_MOTORS]  = {255,255,255,255,255,255 };

static const uint16_t EDGE_LOCKOUT_US = 100;  // ignore spurious edges that are too close
static const uint32_t CTRL_DT_US      = 100;  // outer loop tick

// ADC span for zero-voltage to mechanical-zero mapping
static const float VSPAN = 3.3f;

// ---------------- ISR state per motor ----------------
volatile uint32_t tEdge[NUM_MOTORS] = {0};
volatile uint32_t tPrev[NUM_MOTORS] = {0};
volatile uint32_t period[NUM_MOTORS] = {0};

// ---------------- Calibration: “mechanical zero” per motor (volts) ----------------
volatile float ZERO_V[NUM_MOTORS] = {
  2.29f, 2.21f, 1.66f, 1.59f, 2.94f, 0.82f
};

// ---------------- Targets: per-slave mechanical phase vs reference (cycles) ----------------
// For the REF motor itself, value is ignored. Initialize as you like (e.g., 180° = 0.5)
volatile float PHASE_TARGET_CYC[NUM_MOTORS] = {
  0.00f, 0.00f, 0.25f, 0.25f, 0.00f, 0.00f
};

// ---------------- PID plumbing for 5 slaves ----------------
// We keep arrays matching motor indices; PID objects exist only for non-reference motors.
double PID_Input[NUM_MOTORS]   = {0};
double PID_Output[NUM_MOTORS]  = {0};
double PID_Setpoint[NUM_MOTORS]= {0};

double Kp = 45.0;  // counts per cycle of phase error
double Ki = 40.0;  // counts per cycle per second
double Kd = 0.0;

PID* pids[NUM_MOTORS] = {nullptr};  // only non-null for slaves

// ---------------- Utility: wrapping helpers ----------------
static inline int32_t wrapHalfPeriod(int32_t e_us, uint32_t T_us) {
  if (!T_us) return e_us;
  int32_t half = (int32_t)(T_us / 2u);
  while (e_us >  half) e_us -= (int32_t)T_us;
  while (e_us <= -half) e_us += (int32_t)T_us;
  return e_us;
}

static inline float wrapCycles(float cyc) {
  while (cyc >  0.5f) cyc -= 1.0f;
  while (cyc <= -0.5f) cyc += 1.0f;
  return cyc;
}

// ---------------- ISRs ----------------
void isr0(){ uint32_t t=micros(); if (t - tEdge[0] < EDGE_LOCKOUT_US) return; tPrev[0]=tEdge[0]; tEdge[0]=t; if (tPrev[0]) period[0]=tEdge[0]-tPrev[0]; }
void isr1(){ uint32_t t=micros(); if (t - tEdge[1] < EDGE_LOCKOUT_US) return; tPrev[1]=tEdge[1]; tEdge[1]=t; if (tPrev[1]) period[1]=tEdge[1]-tPrev[1]; }
void isr2(){ uint32_t t=micros(); if (t - tEdge[2] < EDGE_LOCKOUT_US) return; tPrev[2]=tEdge[2]; tEdge[2]=t; if (tPrev[2]) period[2]=tEdge[2]-tPrev[2]; }
void isr3(){ uint32_t t=micros(); if (t - tEdge[3] < EDGE_LOCKOUT_US) return; tPrev[3]=tEdge[3]; tEdge[3]=t; if (tPrev[3]) period[3]=tEdge[3]-tPrev[3]; }
void isr4(){ uint32_t t=micros(); if (t - tEdge[4] < EDGE_LOCKOUT_US) return; tPrev[4]=tEdge[4]; tEdge[4]=t; if (tPrev[4]) period[4]=tEdge[4]-tPrev[4]; }
void isr5(){ uint32_t t=micros(); if (t - tEdge[5] < EDGE_LOCKOUT_US) return; tPrev[5]=tEdge[5]; tEdge[5]=t; if (tPrev[5]) period[5]=tEdge[5]-tPrev[5]; }

typedef void (*isr_fn_t)();
static const isr_fn_t ISR_FUNS[NUM_MOTORS] = { isr0, isr1, isr2, isr3, isr4, isr5 };

// ---------------- Runtime setters ----------------
void setPhaseOffsetCyclesForIndex(uint8_t idx, float cyc){
  if (idx >= NUM_MOTORS || idx == REF_IDX) return;
  noInterrupts();
  PHASE_TARGET_CYC[idx] = wrapCycles(cyc);
  interrupts();
}

void setPhaseOffsetCyclesForPwm(uint8_t pwmPin, float cyc){
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    if (PWM_PINS[i]==pwmPin){ setPhaseOffsetCyclesForIndex(i, cyc); return; }
  }
}

void setZeroVoltageForIndex(uint8_t idx, float v){
  if (idx >= NUM_MOTORS) return;
  noInterrupts();
  ZERO_V[idx] = v;
  interrupts();
}

void setZeroVoltageForPwm(uint8_t pwmPin, float v){
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    if (PWM_PINS[i]==pwmPin){ setZeroVoltageForIndex(i, v); return; }
  }
}

// ---------------- Setup ----------------
void setup(){
  // Resolve reference index
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    if (PWM_PINS[i] == REF_PWM_PIN){ REF_IDX = i; break; }
  }

  // Safety check: must find ref
  if (REF_IDX < 0){
    while(true){
      // Blink LED fast to signal config error
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
  }

  // PWM pins
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    pinMode(PWM_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], PWM_BASE[i]); // start at base open-loop
    PID_Output[i] = PWM_BASE[i];           // initialize PID outputs as well
  }

  // Edge inputs + interrupts
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    pinMode(EDGE_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EDGE_PINS[i]), ISR_FUNS[i], FALLING);
  }

  // PID instances for NON-reference motors
  for (uint8_t i=0;i<NUM_MOTORS;i++){
    if (i == REF_IDX) continue;
    pids[i] = new PID(&PID_Input[i], &PID_Output[i], &PID_Setpoint[i], Kp, Ki, Kd, DIRECT);
    pids[i]->SetOutputLimits(PWM_MIN[i], PWM_MAX[i]);
    pids[i]->SetSampleTime(1);   // 1 ms internal PID update
    pids[i]->SetMode(AUTOMATIC);
    PID_Setpoint[i] = 0.0;       // always regulate e_norm -> 0
  }
}

// ---------------- Main loop ----------------
void loop(){
  static uint32_t tNext = micros();
  static int16_t lastCmd[NUM_MOTORS] = {
    PWM_BASE[0], PWM_BASE[1], PWM_BASE[2], PWM_BASE[3], PWM_BASE[4], PWM_BASE[5]
  };

  if ((int32_t)(micros() - tNext) >= 0){
    tNext += CTRL_DT_US;

    // Snapshot ISR vars & user parameters atomically
    uint32_t T_ref, t_ref;
    uint32_t T[NUM_MOTORS], t[NUM_MOTORS];
    float Z[NUM_MOTORS];
    float target_cyc[NUM_MOTORS];

    noInterrupts();
    T_ref = period[REF_IDX];
    t_ref = tEdge[REF_IDX];
    for (uint8_t i=0;i<NUM_MOTORS;i++){
      T[i] = period[i];
      t[i] = tEdge[i];
      Z[i] = ZERO_V[i];
      target_cyc[i] = PHASE_TARGET_CYC[i];
    }
    interrupts();

    // If we don't yet have valid ref edges: slew all toward base and exit
    if (T_ref == 0 || t_ref == 0){
      const int16_t SLEW = 4;
      for (uint8_t i=0;i<NUM_MOTORS;i++){
        int16_t base = PWM_BASE[i];
        if (base > lastCmd[i] + SLEW)      lastCmd[i] += SLEW;
        else if (base < lastCmd[i] - SLEW) lastCmd[i] -= SLEW;
        else                               lastCmd[i]  = base;
        if (lastCmd[i] < PWM_MIN[i]) lastCmd[i] = PWM_MIN[i];
        if (lastCmd[i] > PWM_MAX[i]) lastCmd[i] = PWM_MAX[i];
        analogWrite(PWM_PINS[i], (uint8_t)lastCmd[i]);
      }
      return;
    }

    // Reference runs open-loop (or you can add a speed loop if you want)
    // Keep it near its base with a mild slew to avoid jumps.
    {
      const int16_t SLEW = 2;
      int16_t base = PWM_BASE[REF_IDX];
      if (base > lastCmd[REF_IDX] + SLEW)      lastCmd[REF_IDX] += SLEW;
      else if (base < lastCmd[REF_IDX] - SLEW) lastCmd[REF_IDX] -= SLEW;
      else                                     lastCmd[REF_IDX]  = base;
      if (lastCmd[REF_IDX] < PWM_MIN[REF_IDX]) lastCmd[REF_IDX] = PWM_MIN[REF_IDX];
      if (lastCmd[REF_IDX] > PWM_MAX[REF_IDX]) lastCmd[REF_IDX] = PWM_MAX[REF_IDX];
      analogWrite(PWM_PINS[REF_IDX], (uint8_t)lastCmd[REF_IDX]);
    }

    // For each SLAVE: compute normalized phase error vs reference and drive PID
    for (uint8_t i=0;i<NUM_MOTORS;i++){
      if (i == REF_IDX) continue;

      // Prefer reference period for normalization
      uint32_t Tnorm = T_ref;
      // If slave or ref timestamps are missing, gracefully slew toward base
      if (t[i] == 0 || t_ref == 0 || Tnorm == 0){
        const int16_t SLEW = 4;
        int16_t base = PWM_BASE[i];
        if (base > lastCmd[i] + SLEW)      lastCmd[i] += SLEW;
        else if (base < lastCmd[i] - SLEW) lastCmd[i] -= SLEW;
        else                               lastCmd[i]  = base;
        if (lastCmd[i] < PWM_MIN[i]) lastCmd[i] = PWM_MIN[i];
        if (lastCmd[i] > PWM_MAX[i]) lastCmd[i] = PWM_MAX[i];
        analogWrite(PWM_PINS[i], (uint8_t)lastCmd[i]);
        continue;
      }

      // ---- Convert desired MECHANICAL phase to EDGE offset (cycles) ----
      // edge_offset = mech_offset + (ZERO_ref - ZERO_i)/VSPAN
      float calib_edge_offset_cyc = (Z[REF_IDX] - Z[i]) / VSPAN;
      float desired_edge_offset_cyc = wrapCycles(target_cyc[i] + calib_edge_offset_cyc);

      // ---- Build normalized error e_norm in (-0.5..+0.5] ----
      // Positive error means ref edge leads slave edge by more than desired (same convention as original)
      int32_t desired_offset_us = (int32_t)lrintf(desired_edge_offset_cyc * (float)Tnorm);
      int32_t e_us = (int32_t)(t_ref - t[i]) - desired_offset_us;
      e_us = wrapHalfPeriod(e_us, Tnorm);
      float e_norm = (float)e_us / (float)Tnorm;  // dimensionless

      // ---- PID step ----
      PID_Setpoint[i] = 0.0;
      PID_Input[i]    = (double)e_norm;

      if (pids[i]->Compute()){
        // Optional slew limiting
        const int16_t SLEW = 4;
        int16_t target = (int16_t)PID_Output[i];
        if (target > lastCmd[i] + SLEW)      lastCmd[i] += SLEW;
        else if (target < lastCmd[i] - SLEW) lastCmd[i] -= SLEW;
        else                                 lastCmd[i]  = target;

        // Safety clamps
        if (lastCmd[i] < PWM_MIN[i]) lastCmd[i] = PWM_MIN[i];
        if (lastCmd[i] > PWM_MAX[i]) lastCmd[i] = PWM_MAX[i];

        analogWrite(PWM_PINS[i], (uint8_t)lastCmd[i]);
      }
    }
  }
}

/* ================= Helper notes =================
- To change a phase target (in cycles) on the fly, e.g. +90° for motor on pin 7:
    setPhaseOffsetCyclesForPwm(7, 0.25f);

- To set mechanical-zero calibration (volts) for a motor on pin 3:
    setZeroVoltageForPwm(3, 1.55f);

- If you want individual PID gains per slave, duplicate Kp/Ki/Kd per motor
  and call pids[i]->SetTunings(Kp_i, Ki_i, Kd_i) after setup or at runtime.

- If your comparator edges are inverted, change FALLING to RISING in attachInterrupt.
- If startup locking is slow, consider seeding lastCmd[REF_IDX] higher or widening PWM_MIN.
================================================== */
