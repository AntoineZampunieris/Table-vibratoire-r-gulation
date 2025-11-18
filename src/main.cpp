#include <Arduino.h>
#include <QuickPID.h>

static const uint8_t NUM_MOTORS = 2;

// // ---------------- Pin maps (your values) ----------------
// static const uint8_t PWM_PINS[NUM_MOTORS]  = {12, 10, 8, 6, 5, 4}; // idx 0..5
// static const uint8_t EDGE_PINS[NUM_MOTORS] = {13, 11, 9, 7, 2, 3}; // idx 0..5
// ---------------- Pin maps (your values) ----------------
static const uint8_t PWM_PINS[NUM_MOTORS]  = {8, 6}; // idx 0..5
static const uint8_t EDGE_PINS[NUM_MOTORS] = {9, 7}; // idx 0..5

// We choose the motor on PWM pin 5 as the global master
static const uint8_t REF_PWM_PIN = 8;
int8_t REF_IDX = -1;   // resolved at setup()

// ---------------- Mechanical zero voltages (your measured values) ----------------
// volatile float ZERO_V[NUM_MOTORS] = {
//   2.29f, 2.21f, 1.22f, 1.66f, 0.82f, 2.94f
// } ;

// const float VSPAN = 3.3f;   // sensor full-scale

// // ---------------- Fixed settings ----------------
// const uint8_t PWM_BASE[NUM_MOTORS] = {
//   180, 180, 180, 180, 180, 180
// };

// const uint8_t PWM_MIN[NUM_MOTORS] = {
//   20, 20, 20, 20, 20, 20
// };

// const uint8_t PWM_MAX[NUM_MOTORS] = {
//   255, 255, 255, 255, 255, 255
// };
volatile float ZERO_V[NUM_MOTORS] = {
  1.22f, 1.66f
} ;

const float VSPAN = 3.3f;   // sensor full-scale

// ---------------- Fixed settings ----------------
const uint8_t PWM_BASE[NUM_MOTORS] = {
  200, 200
};

const uint8_t PWM_MIN[NUM_MOTORS] = {
  20, 20
};

const uint8_t PWM_MAX[NUM_MOTORS] = {
  255, 255
};

const uint16_t EDGE_LOCKOUT_US = 100;   // ignore edges that are too close
const uint32_t CTRL_DT_US      = 0;   // outer control loop period

// ---------------- Desired mechanical phase vs MASTER (in cycles) ----------------
// positive = MASTER leads this motor by this fraction of a cycle
// (0.25 = +90°, 0.5 = +180°). For master itself, entry is ignored.
// volatile float phaseOffset_cycles[NUM_MOTORS] = {
//   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
// };
volatile float phaseOffset_cycles[NUM_MOTORS] = {
  0.00f, 0.00f
};

// ---------------- ISR state ----------------
volatile uint32_t tEdge[NUM_MOTORS]  = {0};
volatile uint32_t tPrev[NUM_MOTORS]  = {0};
volatile uint32_t period[NUM_MOTORS] = {0};

void isr0() {
  uint32_t t = micros();
  if (t - tEdge[0] < EDGE_LOCKOUT_US) return;
  tPrev[0] = tEdge[0]; tEdge[0] = t;
  if (tPrev[0]) period[0] = tEdge[0] - tPrev[0];
}
void isr1() {
  uint32_t t = micros();
  if (t - tEdge[1] < EDGE_LOCKOUT_US) return;
  tPrev[1] = tEdge[1]; tEdge[1] = t;
  if (tPrev[1]) period[1] = tEdge[1] - tPrev[1];
}
void isr2() {
  uint32_t t = micros();
  if (t - tEdge[2] < EDGE_LOCKOUT_US) return;
  tPrev[2] = tEdge[2]; tEdge[2] = t;
  if (tPrev[2]) period[2] = tEdge[2] - tPrev[2];
}
void isr3() {
  uint32_t t = micros();
  if (t - tEdge[3] < EDGE_LOCKOUT_US) return;
  tPrev[3] = tEdge[3]; tEdge[3] = t;
  if (tPrev[3]) period[3] = tEdge[3] - tPrev[3];
}
void isr4() {
  uint32_t t = micros();
  if (t - tEdge[4] < EDGE_LOCKOUT_US) return;
  tPrev[4] = tEdge[4]; tEdge[4] = t;
  if (tPrev[4]) period[4] = tEdge[4] - tPrev[4];
}
void isr5() {
  uint32_t t = micros();
  if (t - tEdge[5] < EDGE_LOCKOUT_US) return;
  tPrev[5] = tEdge[5]; tEdge[5] = t;
  if (tPrev[5]) period[5] = tEdge[5] - tPrev[5];
}

// typedef void (*isr_fn_t)();
// static const isr_fn_t ISR_FUNS[NUM_MOTORS] = {
//   isr0, isr1, isr2, isr3, isr4, isr5
// };
typedef void (*isr_fn_t)();
static const isr_fn_t ISR_FUNS[NUM_MOTORS] = {
  isr0, isr1
};

// ---------------- Helpers: wrapping ----------------
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

// ---------------- PID plumbing: one PID per SLAVE motor ----------------
float PID_Input[NUM_MOTORS]    = {0.0f};
float PID_Output[NUM_MOTORS]   = {0.0f};
float PID_Setpoint[NUM_MOTORS] = {0.0f};

// Gains (start from the values that worked for your single pair)
double Kp = 80.0;
double Ki = 80.0;
double Kd = 0.0;

QuickPID* pids[NUM_MOTORS] = { nullptr };


// Slew-limited last commands per motor
int16_t lastCmd[NUM_MOTORS];

// Current selected mode (0 or 1)
uint8_t currentMode = 0;

// ------------- Serial command buffer -------------
static char cmdBuf[32];
static uint8_t cmdIdx = 0;

// ---------------- Runtime helpers ----------------
inline void setPhaseOffsetForMotor(uint8_t idx, float cyc) {
  if (idx >= NUM_MOTORS || idx == REF_IDX) return;  // ignore master
  noInterrupts();
  phaseOffset_cycles[idx] = wrapCycles(cyc);
  interrupts();
}

// Helper to set phase offset by PWM pin, not by index
void setPhaseOffsetForPwm(uint8_t pwmPin, float cyc) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (PWM_PINS[i] == pwmPin) {
      setPhaseOffsetForMotor(i, cyc);
      return;
    }
  }
}

// Zero voltage setter, if needed
inline void setZeroVoltageForMotor(uint8_t idx, float v) {
  if (idx >= NUM_MOTORS) return;
  noInterrupts();
  ZERO_V[idx] = v;
  interrupts();
}

// Update tunings on all PIDs (slaves)
void updateAllTunings() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (i == REF_IDX) continue;
    if (pids[i]) {
      pids[i]->SetTunings(Kp, Ki, Kd);
    }
  }
  Serial.print(F("New gains: Kp="));
  Serial.print(Kp, 4);
  Serial.print(F(", Ki="));
  Serial.print(Ki, 4);
  Serial.print(F(", Kd="));
  Serial.println(Kd, 4);
}

// Apply operating mode:
// mode 0: all phase offsets = 0
// mode 1: all = 0, except PWM 8 and PWM 6 at 0.33 cycles
void applyMode(uint8_t mode) {
  noInterrupts();
  // reset all to 0
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (i == REF_IDX) continue;
    phaseOffset_cycles[i] = 0.0f;
  }
  interrupts();

  if (mode == 1) {
    // set PWM 8 and 6 to 0.33 cycles
    setPhaseOffsetForPwm(8, 0.33f);
    setPhaseOffsetForPwm(6, 0.33f);
  }

  currentMode = mode;

  Serial.print(F("Mode set to "));
  Serial.println(mode);
  if (mode == 0) {
    Serial.println(F("All phase offsets = 0"));
  } else if (mode == 1) {
    Serial.println(F("All phase offsets = 0, except PWM 8 & 6 = 0.33 cycles"));
  }
}

// Simple lowercase helper
char toLowerChar(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A' + 'a';
  return c;
}

// Process a full line from Serial
void processCommand(const char* line) {
  // Make a small, mutable copy
  char buf[32];
  strncpy(buf, line, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  // Trim leading spaces
  char* p = buf;
  while (*p == ' ' || *p == '\t') p++;

  // Lowercase first word for easier parsing
  for (char* q = p; *q != '\0' && *q != ' ' && *q != '\t'; ++q) {
    *q = toLowerChar(*q);
  }

  // Mode change: "0", "1", "mode 0", "mode 1"
  if ((p[0] == '0' && p[1] == '\0') || strncmp(p, "mode 0", 6) == 0) {
    applyMode(0);
    return;
  }
  if ((p[0] == '1' && p[1] == '\0') || strncmp(p, "mode 1", 6) == 0) {
    applyMode(1);
    return;
  }

  // Gains: "kp 10.5", "ki 20", "kd 0.0"
  if (strncmp(p, "kp", 2) == 0) {
    char* valStr = p + 2;
    while (*valStr == ' ' || *valStr == '\t' || *valStr == '=') valStr++;
    if (*valStr != '\0') {
      Kp = atof(valStr);
      updateAllTunings();
    }
    return;
  }

  if (strncmp(p, "ki", 2) == 0) {
    char* valStr = p + 2;
    while (*valStr == ' ' || *valStr == '\t' || *valStr == '=') valStr++;
    if (*valStr != '\0') {
      Ki = atof(valStr);
      updateAllTunings();
    }
    return;
  }

  if (strncmp(p, "kd", 2) == 0) {
    char* valStr = p + 2;
    while (*valStr == ' ' || *valStr == '\t' || *valStr == '=') valStr++;
    if (*valStr != '\0') {
      Kd = atof(valStr);
      updateAllTunings();
    }
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(line);
  Serial.println(F("Valid: 0, 1, mode 0, mode 1, kp <val>, ki <val>, kd <val>"));
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial on Zero */ }

  Serial.println(F("\n=== Multi-motor phase control ==="));
  Serial.println(F("Master: PWM 5"));
  Serial.println(F("Modes:"));
  Serial.println(F("  0 or 'mode 0' -> all phase offsets = 0"));
  Serial.println(F("  1 or 'mode 1' -> all 0 except PWM 8 & 6 = 0.33 cycles\n"));
  Serial.println(F("Gains:"));
  Serial.println(F("  kp <value>   (e.g. 'kp 10.0')"));
  Serial.println(F("  ki <value>   (e.g. 'ki 20')"));
  Serial.println(F("  kd <value>   (e.g. 'kd 0.0')\n"));

  analogWriteResolution(8); // 0..255 on Arduino Zero

  // Resolve reference index based on REF_PWM_PIN
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (PWM_PINS[i] == REF_PWM_PIN) {
      REF_IDX = i;
      break;
    }
  }

  // Safety: if not found, blink and halt
  if (REF_IDX < 0) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // Configure PWM outputs and initialize lastCmd
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], PWM_BASE[i]);
    lastCmd[i] = PWM_BASE[i];
  }

  // Configure edge inputs and interrupts
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(EDGE_PINS[i], INPUT_PULLUP);  // open-collector comparators
    attachInterrupt(digitalPinToInterrupt(EDGE_PINS[i]), ISR_FUNS[i], FALLING);
  }

  // Create PID controllers for all SLAVES (skip master)
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (i == REF_IDX) {
      pids[i] = nullptr;
      continue;
    }
    PID_Output[i]   = PWM_BASE[i];
    PID_Setpoint[i] = 0.0;
    // Construct QuickPID.
    // Action::direct = same idea as DIRECT (positive error -> increase output).
    pids[i] = new QuickPID(&PID_Input[i], &PID_Output[i], &PID_Setpoint[i],Kp, Ki, Kd, QuickPID::Action::direct);

    pids[i]->SetOutputLimits(PWM_MIN[i], PWM_MAX[i]);
    // Enable automatic mode
    pids[i]->SetMode(QuickPID::Control::automatic);
  }

  // Default mode at startup: 0 (all in phase)
  applyMode(0);
  updateAllTunings();
}

// ---------------- Main control loop ----------------
void loop() {
  // --- Handle Serial input (line-based) ---
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue; // ignore CR
    if (c == '\n') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else {
      if (cmdIdx < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdIdx++] = c;
      }
    }
  }

  // --- Phase control loop ---
  static uint32_t tNext = micros();

  if ((int32_t)(micros() - tNext) >= 0) {
    tNext += CTRL_DT_US;

    // Snapshot ISR & calibration vars atomically
    uint32_t T[NUM_MOTORS];
    uint32_t t[NUM_MOTORS];
    float    Z[NUM_MOTORS];
    float    off_cyc[NUM_MOTORS];

    noInterrupts();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      T[i] = period[i];
      t[i] = tEdge[i];
      Z[i] = ZERO_V[i];
      off_cyc[i] = phaseOffset_cycles[i];
    }
    interrupts();

    // Reference data
    uint32_t TA_ref = t[REF_IDX];
    uint32_t T_ref  = T[REF_IDX];

    // If the master isn't giving valid edges yet: slew everyone to base
    if (T_ref == 0 || TA_ref == 0) {
      const int16_t SLEW = 4;
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
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

    // Keep master near its base (open-loop)
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

    // Regulate each SLAVE relative to the master
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (i == REF_IDX) continue;  // skip master

      uint32_t Tnorm = T_ref;

      // If this slave has no valid edges yet: gently go to base
      if (t[i] == 0 || Tnorm == 0) {
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

      // ---- Convert desired MECHANICAL phase to required EDGE offset (cycles) ----
      // edge_offset = mech_offset + (ZERO_master - ZERO_i) / VSPAN
      float calib_edge_offset_cyc = (Z[REF_IDX] - Z[i]) / VSPAN;
      float desired_edge_offset_cyc = wrapCycles(off_cyc[i] + calib_edge_offset_cyc);

      // ---- Build normalized error e_norm in (-0.5..+0.5] ----
      int32_t desired_offset_us = (int32_t)lrintf(desired_edge_offset_cyc * (float)Tnorm);
      int32_t e_us = (int32_t)(TA_ref - t[i]) - desired_offset_us;
      e_us = wrapHalfPeriod(e_us, Tnorm);
      float e_norm = (float)e_us / (float)Tnorm;

      // ---- Feed PID ----
      PID_Input[i]    = (double)e_norm;
      PID_Setpoint[i] = 0.0;

      if (pids[i] && pids[i]->Compute()) {
        const int16_t SLEW = 4;
        int16_t target = (int16_t)PID_Output[i];

        if (target > lastCmd[i] + SLEW)      lastCmd[i] += SLEW;
        else if (target < lastCmd[i] - SLEW) lastCmd[i] -= SLEW;
        else                                 lastCmd[i]  = target;

        if (lastCmd[i] < PWM_MIN[i]) lastCmd[i] = PWM_MIN[i];
        if (lastCmd[i] > PWM_MAX[i]) lastCmd[i] = PWM_MAX[i];

        analogWrite(PWM_PINS[i], (uint8_t)lastCmd[i]);
      }
    }
  }
}
