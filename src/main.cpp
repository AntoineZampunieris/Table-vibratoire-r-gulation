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
/* ---------------- Constants ----------------
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
}*/

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

// static const uint8_t NUM_MOTORS = 6;

// // ---------------- Pin maps ----------------
// static const uint8_t PWM_PINS[NUM_MOTORS]  = {3, 4, 5, 6, 7, 8};             // idx 0..5
// static const uint8_t EDGE_PINS[NUM_MOTORS] = {A5, A4, A2, A3, A0, A1};       // idx 0..5

// // Reference motor is the one on PWM pin 5 (preferably)
// static const uint8_t REF_PWM_PIN = 5;
// static int8_t REF_IDX = -1;   // resolved at setup()

// // ---------------- Fixed settings ----------------
// static const uint8_t PWM_BASE[NUM_MOTORS] = {
//   150, 150, 150, 150, 150, 150  // reference can have a slightly different base if desired
// };
// static const uint8_t PWM_MIN[NUM_MOTORS]  = { 20, 20, 20, 20, 20, 20 };
// static const uint8_t PWM_MAX[NUM_MOTORS]  = {255,255,255,255,255,255 };

// static const uint16_t EDGE_LOCKOUT_US = 100;  // ignore spurious edges that are too close
// static const uint32_t CTRL_DT_US      = 100;  // outer loop tick

// // ADC span for zero-voltage to mechanical-zero mapping
// static const float VSPAN = 3.3f;

// // ---------------- ISR state per motor ----------------
// volatile uint32_t tEdge[NUM_MOTORS] = {0};
// volatile uint32_t tPrev[NUM_MOTORS] = {0};
// volatile uint32_t period[NUM_MOTORS] = {0};

// // ---------------- Calibration: “mechanical zero” per motor (volts) ----------------
// volatile float ZERO_V[NUM_MOTORS] = {
//   2.29f, 2.21f, 1.66f, 1.59f, 2.94f, 0.82f
// };

// // ---------------- Targets: per-slave mechanical phase vs reference (cycles) ----------------
// // For the REF motor itself, value is ignored. Initialize as you like (e.g., 180° = 0.5)
// volatile float PHASE_TARGET_CYC[NUM_MOTORS] = {
//   0.00f, 0.00f, 0.25f, 0.25f, 0.00f, 0.00f
// };


// // Change these to whatever pins you are using:
// const uint8_t PWM_PIN = 10;
// const uint8_t IN_PIN  = 11;   // noisy edge signal in
// const uint8_t OUT_PIN = 2;    // clean digital out to observe on scope

// volatile bool outState = LOW;

// void edgeISR() {
//   // Toggle the output pin every time an interrupt is triggered
//   outState = !outState;
//   digitalWrite(OUT_PIN, outState);
// }

// void setup() {

//   // PWM pins
//   // for (uint8_t i=0;i<NUM_MOTORS;i++){
//   //   pinMode(PWM_PINS[i], OUTPUT);
//   //   analogWrite(PWM_PINS[i], PWM_BASE[i]); // start at base open-loop
//   // }
//   pinMode(PWM_PIN, OUTPUT);
//   analogWrite(PWM_PIN, 150); // start at base open-loop
//   // Use the digital input buffer on IN_PIN
//   pinMode(IN_PIN, INPUT);   // or INPUT if you already have a pull-up

//   pinMode(OUT_PIN, OUTPUT);
//   digitalWrite(OUT_PIN, LOW);

//   // Interrupt on falling edges of IN_PIN
//   attachInterrupt(digitalPinToInterrupt(IN_PIN), edgeISR, FALLING);
// }

// void loop() {
//   // Nothing; everything happens in the ISR
// }

// #include <Arduino.h>
 
 
// // --- Définitions des constantes ---
 
// #define REF_INTERRUPT_PIN A0
// // Broche de sortie pour générer un créneau à chaque passage à zéro
// #define REF_PULSE_PIN 9

// // Durée du créneau en microsecondes
 
 
 
 
// // --- Variables moteur maître ---
// volatile unsigned long lastRefTime = 0;
// volatile unsigned long refPeriod = 1; //REF_PERIOD_DEFAULT;
// volatile bool refDetected = false;
// // Flags pour gérer la génération du créneau sans appeler digitalWrite() dans l'ISR
// // volatile bool pulseRequest = false;
// // volatile unsigned long pulseRequestTime = 0;
// // volatile bool pulseActive = false;
// // volatile unsigned long pulseEndTime = 0;
 
// // --- Variables moteur esclave ---
// unsigned long lastAngleTime = 0;
// float lastAngle = 0;
 
// // --- Commandes utilisateur ---
// int userSpeed = 50;
// bool motorRunning = true;
// // Inversion par défaut pour chaque moteur. motor1 était précédemment inversé dans le code.
// bool motor1Inverted = true;
// bool motor2Inverted = false;
// bool debugMotors = false;
// // Affiche le KPI (Kp) initial au démarrage si besoin
 
// // --- Prototypes ---
 
// void isrMoteurMaitre();
 
// void testInterruption();
 
// void setup()
// {
//   Serial.begin(115200);
 
 
//   // Interruption sur pin de référence moteur maître
//   pinMode(REF_INTERRUPT_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(REF_INTERRUPT_PIN), isrMoteurMaitre, FALLING);
 
//   // Broche de sortie pour le créneau de référence
//   // pinMode(REF_PULSE_PIN, OUTPUT);
 
//   // digitalWrite(REF_PULSE_PIN, LOW);
 
//   //   pinMode(REF_INTERRUPT_PIN, INP);
//   // REF_INTERRUPT_PIN
 
 
// }
 
// void loop()
// {
 
 
// //   if (digitalRead(REF_INTERRUPT_PIN) == 0)
// //   {
// //     digitalWrite(REF_PULSE_PIN, LOW); // Activer le pulse
// //   }
// //   else
 
// //     digitalWrite(REF_PULSE_PIN, digitalRead(REF_INTERRUPT_PIN)); // Activer le pulse
 
 
// // }
 
// //digitalWrite(REF_PULSE_PIN, digitalRead(REF_INTERRUPT_PIN)); // Activer le pulse
// }
 
// // --- Gestion des commandes série ---
 
 
// // --- Interruption passage à zéro moteur maître ---
// void isrMoteurMaitre()
// {
 
//   testInterruption();// unsigned long now = micros();
//   // refPeriod = now - lastRefTime;
//   // lastRefTime = now;
//    refDetected = true;
// }
 
 
 
 
 
// // --- Test interruption ---
// void testInterruption()
// {
//   static unsigned long lastPrint = 0;
//   if (refDetected == true)
//   {
//     digitalWrite(REF_PULSE_PIN, !digitalRead(REF_PULSE_PIN)); // Activer le pulse
//     refDetected = false;
//   }
// }

#include <Arduino.h>
#include <PID_v1.h>

// ===== Pins =====
const uint8_t PIN_EDGE_A = 7;   // Comparator A -> D2 (master)
const uint8_t PIN_EDGE_B = 9;   // Comparator B -> D3 (slave)

const uint8_t MASTER_PWM = 6;   // Motor A PWM
//const uint8_t MASTER_DIR = 4;   // Motor A DIR (set level you use for forward)
const uint8_t SLAVE_PWM  = 8;   // Motor B PWM
//const uint8_t SLAVE_DIR  = 7;   // Motor B DIR (set level you use for forward)

// ===== Fixed settings =====
const uint8_t MASTER_PWM_BASE = 150;  // open-loop base for master (0..255)
const uint8_t SLAVE_PWM_BASE  = 150;  // nominal base for slave (0..255)
const uint8_t SLAVE_PWM_MIN   = 20;  // helps overcome static friction
const uint8_t SLAVE_PWM_MAX   = 255;

const uint16_t EDGE_LOCKOUT_US = 100;   // ignore edges that are too close
const uint32_t CTRL_DT_US      = 100;   // control loop period (µs) -- we'll run fast; PID runs at 1 ms

// ===== Desired mechanical phase offset =====
// positive = A leads B by this fraction of a cycle (0.25 = +90°, 0.5 = 180°)
volatile float phaseOffset_cycles = 0.00f;

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
double Kp = 80.0;   // start here, then tune
double Ki = 20.0;   // integral strength (per second)
double Kd = 0.0;

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
