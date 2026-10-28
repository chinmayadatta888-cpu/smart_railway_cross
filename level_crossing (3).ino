/*
  Smart Railway Level-Crossing Safety Controller with Emergency Override
  Arduino Uno | scale-model prototype, NOT a certified railway safety system

  Features
   - Axle counting with two FSRs (entry adds, exit subtracts)
   - Speed estimate -> constant warning time (hold-off before warning starts)
   - Non-blocking millis() FSM (no delay() after setup)
   - Emergency button on hardware interrupt (D2 / INT0)
   - Fault detection: open circuit, stuck sensor, exit-before-entry
   - Stalled-train alarm, smooth servo barrier, distinct buzzer patterns
   - Serial event log

  Pin map (matches the wiring guide)
   A0 entry FSR | A1 exit FSR | D2 emergency button (to GND, INPUT_PULLUP)
   D3 road GREEN pair | D4 road YELLOW pair | D5 road RED pair
   D6 / D7 railway LEDs | D8 -> 220 ohm -> piezo + | D9 servo
*/

#include <Servo.h>

// ---------------- Pins ----------------
const uint8_t PIN_ENTRY  = A0;
const uint8_t PIN_EXIT   = A1;
const uint8_t PIN_EMERG  = 2;
const uint8_t PIN_ROAD_G = 3;
const uint8_t PIN_ROAD_Y = 4;
const uint8_t PIN_ROAD_R = 5;
const uint8_t PIN_WARN1  = 6;
const uint8_t PIN_WARN2  = 7;
const uint8_t PIN_BUZZ   = 8;
const uint8_t PIN_SERVO  = 9;

// ---------------- Sensor tuning (check with the Serial Monitor) ----------------
const int PRESS_ON  = 300;   // ADC above this = pressed
const int PRESS_OFF = 150;   // ADC below this = released (hysteresis)
const int OPEN_LIMIT = 3;    // ADC below this = broken wire (needs the 470k bypass)
#define ENABLE_OPEN_CIRCUIT_CHECK 1   // set to 0 if your idle reading is ~0
const unsigned long SENSOR_DEBOUNCE_MS = 40;
const unsigned long FAULT_CONFIRM_MS   = 300;
const unsigned long STUCK_MS           = 30000UL;  // pressed this long = stuck

// ---------------- Model geometry (change to match your model) ----------------
const unsigned long AXLE_SPACING_MM      = 60;   // between first two axles of a train
const unsigned long DIST_TO_CROSSING_MM  = 600;  // entry sensor -> crossing
const unsigned long SENSOR_SPACING_MM    = 800;  // entry sensor -> exit sensor

// ---------------- Timing ----------------
const unsigned long YELLOW_MS            = 2000;
const unsigned long TARGET_WARNING_MS    = 5000;   // warning lead time before arrival
const unsigned long SPEED_WAIT_MS        = 1500;   // no 2nd axle by now -> warn anyway
const unsigned long MAX_HOLDOFF_MS       = 15000;
const unsigned long EXIT_HOLD_MS         = 1500;   // quiet time after last axle leaves
const unsigned long OCCUPIED_TIMEOUT_MS  = 30000UL;
const unsigned long FLASH_MS             = 300;
const unsigned long BARRIER_STEP_MS      = 15;     // 1 degree per step

// Fast-train mode: when the ETA is too short for the normal sequence,
// use a shorter yellow phase and a faster barrier.
const unsigned long FAST_YELLOW_MS = 1000;
const unsigned long FAST_STEP_MS   = 5;      // 90 steps = 0.45 s of barrier travel
const unsigned long FAST_MARGIN_MS = 1000;   // spare time wanted before arrival

const int BARRIER_OPEN   = 90;
const int BARRIER_CLOSED = 0;   // swap the two values if your arm moves the wrong way

// ---------------- Constants ----------------
enum { S_CLEAR, S_APPROACH, S_WARNING, S_CLOSING, S_OCCUPIED,
       S_EXITING, S_CLEARING, S_FAULT, S_EMERGENCY };
const char* const STATE_NAMES[] = { "CLEAR", "APPROACH", "WARNING", "CLOSING",
  "CROSSING OCCUPIED", "EXITING", "CLEARING", "FAULT", "EMERGENCY" };

enum { ROAD_GREEN, ROAD_YELLOW, ROAD_RED };
enum { BZ_OFF, BZ_WARN, BZ_EMERG, BZ_FAULT, BZ_STALLED };

// ---------------- Sensors ----------------
struct Sensor {
  uint8_t pin;
  int value;
  bool pressed;
  unsigned long lastChange;
  bool low;
  unsigned long lowStart;
  bool openCircuit;
  bool stuck;
};
Sensor sensors[2] = {
  { PIN_ENTRY, 0, false, 0, false, 0, false, false },
  { PIN_EXIT,  0, false, 0, false, 0, false, false }
};
const uint8_t ENTRY = 0, EXIT_S = 1;

// ---------------- Globals ----------------
Servo barrier;
int barrierPos = BARRIER_OPEN;
int barrierTarget = BARRIER_OPEN;

uint8_t state = S_CLEAR;
uint8_t roadMode = ROAD_GREEN;
uint8_t buzz = BZ_OFF;
bool flash = false;
bool safeYellow = false;
bool stalled = false;

int axlesIn = 0, axlesOut = 0;
unsigned long tState = 0, warnAt = 0, tFirstIn = 0, lastExitMs = 0;
unsigned long closureStart = 0, transitMs = 0;
unsigned int trainCount = 0, faultCount = 0;
unsigned long yellowMs = YELLOW_MS;          // active yellow duration
unsigned long stepMs = BARRIER_STEP_MS;      // active barrier step time

// ---------------- Emergency interrupt ----------------
volatile bool emergFlag = false;
volatile unsigned long lastEmergMs = 0;

void emergISR() {
  unsigned long t = millis();
  if (t - lastEmergMs > 300) {   // simple debounce
    emergFlag = true;
    lastEmergMs = t;
  }
}

bool takeEmergPress() {
  noInterrupts();
  bool f = emergFlag;
  emergFlag = false;
  interrupts();
  return f;
}

// ---------------- Outputs ----------------
void setRoad(uint8_t m) {
  roadMode = m;
  digitalWrite(PIN_ROAD_G, m == ROAD_GREEN);
  digitalWrite(PIN_ROAD_Y, m == ROAD_YELLOW);
  digitalWrite(PIN_ROAD_R, m == ROAD_RED);
}

void updateBarrier(unsigned long now) {
  static unsigned long last = 0;
  if (now - last < stepMs) return;
  last = now;
  if (barrierPos < barrierTarget) barrierPos++;
  else if (barrierPos > barrierTarget) barrierPos--;
  else return;
  barrier.write(barrierPos);
}

void updateFlashers(unsigned long now) {
  static unsigned long last = 0;
  static bool phase = false;
  if (!flash) {
    digitalWrite(PIN_WARN1, LOW);
    digitalWrite(PIN_WARN2, LOW);
    return;
  }
  if (now - last >= FLASH_MS) {
    last = now;
    phase = !phase;
  }
  digitalWrite(PIN_WARN1, phase);
  digitalWrite(PIN_WARN2, !phase);
}

void updateBuzzer(unsigned long now) {
  static bool lastOn = false;
  static unsigned int lastFreq = 0;
  bool on = false;
  unsigned int freq = 1000;

  switch (buzz) {
    case BZ_WARN:    on = (now % 600) < 150; freq = 1000; break;           // slow beep
    case BZ_EMERG:   on = (now % 200) < 100; freq = 2000; break;           // fast beep
    case BZ_FAULT:   on = true;              freq = 500;  break;           // long low tone
    case BZ_STALLED: on = true; freq = ((now / 250) % 2) ? 800 : 1500; break; // siren
    default:         on = false; break;
  }

  if (on) {
    if (!lastOn || freq != lastFreq) tone(PIN_BUZZ, freq);
  } else if (lastOn) {
    noTone(PIN_BUZZ);
  }
  lastOn = on;
  lastFreq = freq;
}

// ---------------- Sensor handling ----------------
// Returns true on a debounced rising edge (one axle).
bool updateSensor(uint8_t i, unsigned long now) {
  Sensor &s = sensors[i];
  s.value = analogRead(s.pin);

  // open-circuit detection
  if (s.value < OPEN_LIMIT) {
    if (!s.low) { s.low = true; s.lowStart = now; }
    if (now - s.lowStart >= FAULT_CONFIRM_MS) s.openCircuit = true;
  } else {
    s.low = false;
    s.openCircuit = false;
  }
#if !ENABLE_OPEN_CIRCUIT_CHECK
  s.openCircuit = false;
#endif

  // press / release with hysteresis and debounce
  bool rising = false;
  if (!s.pressed && s.value > PRESS_ON && now - s.lastChange > SENSOR_DEBOUNCE_MS) {
    s.pressed = true;
    s.lastChange = now;
    rising = true;
  } else if (s.pressed && s.value < PRESS_OFF && now - s.lastChange > SENSOR_DEBOUNCE_MS) {
    s.pressed = false;
    s.lastChange = now;
  }

  s.stuck = s.pressed && (now - s.lastChange > STUCK_MS);
  return rising;
}

bool sensorsHealthy() {
  return !sensors[ENTRY].openCircuit && !sensors[EXIT_S].openCircuit &&
         !sensors[ENTRY].stuck && !sensors[EXIT_S].stuck;
}

// ---------------- State helpers ----------------
void setState(uint8_t s) {
  state = s;
  tState = millis();
  if (s == S_WARNING) closureStart = tState;
  Serial.print(F("-> "));
  Serial.println(STATE_NAMES[s]);
}

void resetCounters() {
  axlesIn = 0;
  axlesOut = 0;
  stalled = false;
  transitMs = 0;
}

// Go to a safe state (FAULT or EMERGENCY). If the road is not red yet,
// it passes through yellow first instead of jumping straight to red.
void enterSafe(uint8_t s, const __FlashStringHelper* why) {
  safeYellow = (roadMode != ROAD_RED);
  if (s == S_FAULT) faultCount++;
  Serial.print(F("!! "));
  Serial.println(why);
  setState(s);
}

bool balanced() {
  return axlesIn > 0 && axlesOut >= axlesIn;
}

void onEntryAxle(unsigned long now) {
  if (state == S_CLEARING) {            // new train while reopening
    axlesIn = 1;
    axlesOut = 0;
    stalled = false;
    Serial.println(F("New train while clearing, closing again"));
    setState(S_CLOSING);
    return;
  }

  axlesIn++;
  Serial.print(F("Entry axle, in="));
  Serial.println(axlesIn);

  if (state == S_CLEAR) {
    tFirstIn = now;
    yellowMs = YELLOW_MS;               // normal timing until proven fast
    stepMs = BARRIER_STEP_MS;
    warnAt = now + SPEED_WAIT_MS;       // fallback if no 2nd axle arrives
    setState(S_APPROACH);
  } else if (state == S_APPROACH && axlesIn == 2) {
    // speed from the first two axles (known spacing)
    unsigned long dt = now - tFirstIn;
    if (dt == 0) dt = 1;
    unsigned long speed = AXLE_SPACING_MM * 1000UL / dt;       // mm/s
    unsigned long eta = DIST_TO_CROSSING_MM * dt / AXLE_SPACING_MM;  // ms
    unsigned long hold = (eta > TARGET_WARNING_MS) ? (eta - TARGET_WARNING_MS) : 0;
    if (hold > MAX_HOLDOFF_MS) hold = MAX_HOLDOFF_MS;

    // Fast train: the normal sequence (yellow + barrier travel) would finish
    // too close to, or after, the arrival time, so compress it.
    unsigned long travel = (unsigned long)abs(BARRIER_OPEN - BARRIER_CLOSED);
    unsigned long normalSeq = YELLOW_MS + travel * BARRIER_STEP_MS;
    if (eta < normalSeq + FAST_MARGIN_MS) {
      yellowMs = FAST_YELLOW_MS;
      stepMs = FAST_STEP_MS;
      Serial.println(F("Fast train: short yellow, fast barrier"));
      if (eta < FAST_YELLOW_MS + travel * FAST_STEP_MS) {
        Serial.println(F("!! Too fast for this sensor distance: barrier will be late"));
      }
    }
    warnAt = tFirstIn + hold;   // counted from the FIRST axle, which reaches the crossing first
    long wait = (long)(warnAt - now);
    if (wait < 0) wait = 0;
    Serial.print(F("Speed ~"));  Serial.print(speed);
    Serial.print(F(" mm/s, ETA ")); Serial.print(eta);
    Serial.print(F(" ms, warning starts in ")); Serial.print(wait);
    Serial.println(F(" ms"));
  }
}

void onExitAxle(unsigned long now) {
  if (axlesOut >= axlesIn) {            // exit pulse with nothing inside
    enterSafe(S_FAULT, F("FAULT: exit sensor before entry"));
    return;
  }
  axlesOut++;
  lastExitMs = now;
  if (axlesOut == 1) transitMs = now - tFirstIn;
  Serial.print(F("Exit axle, out="));
  Serial.println(axlesOut);
}

void logTrain(unsigned long now) {
  trainCount++;
  unsigned long speed = transitMs ? SENSOR_SPACING_MM * 1000UL / transitMs : 0;
  Serial.println(F("------ TRAIN LOG ------"));
  Serial.print(F("Train #"));          Serial.println(trainCount);
  Serial.print(F("Axles: "));          Serial.println(axlesIn);
  Serial.print(F("Transit (ms): "));   Serial.println(transitMs);
  Serial.print(F("Speed (mm/s): "));   Serial.println(speed);
  Serial.print(F("Closed for (ms): ")); Serial.println(now - closureStart);
  Serial.print(F("Faults so far: "));  Serial.println(faultCount);
  Serial.println(F("-----------------------"));
}

void printStatus(unsigned long now) {
  static unsigned long last = 0;
  if (now - last < 1000) return;
  last = now;
  Serial.print(F("["));  Serial.print(STATE_NAMES[state]);
  Serial.print(F("] A0=")); Serial.print(sensors[ENTRY].value);
  Serial.print(F(" A1="));  Serial.print(sensors[EXIT_S].value);
  Serial.print(F(" in="));  Serial.print(axlesIn);
  Serial.print(F(" out=")); Serial.print(axlesOut);
  Serial.print(F(" barrier=")); Serial.println(barrierPos);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);

  pinMode(PIN_ROAD_G, OUTPUT);
  pinMode(PIN_ROAD_Y, OUTPUT);
  pinMode(PIN_ROAD_R, OUTPUT);
  pinMode(PIN_WARN1, OUTPUT);
  pinMode(PIN_WARN2, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_EMERG, INPUT_PULLUP);

  barrier.attach(PIN_SERVO);
  barrier.write(BARRIER_OPEN);

  // Lamp test (only blocking code in the sketch, runs once at power-up)
  digitalWrite(PIN_ROAD_G, HIGH);
  digitalWrite(PIN_ROAD_Y, HIGH);
  digitalWrite(PIN_ROAD_R, HIGH);
  digitalWrite(PIN_WARN1, HIGH);
  digitalWrite(PIN_WARN2, HIGH);
  tone(PIN_BUZZ, 1000, 200);
  delay(800);
  digitalWrite(PIN_WARN1, LOW);
  digitalWrite(PIN_WARN2, LOW);

  setRoad(ROAD_GREEN);
  attachInterrupt(digitalPinToInterrupt(PIN_EMERG), emergISR, FALLING);

  Serial.println(F("Level crossing controller ready"));
  setState(S_CLEAR);
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long now = millis();

  bool emergPress = takeEmergPress();
  bool inRise  = updateSensor(ENTRY, now);
  bool outRise = updateSensor(EXIT_S, now);

  // ----- Emergency button -----
  if (emergPress) {
    if (state == S_EMERGENCY) {
      // Operator confirms the crossing is clear -> reopen through CLEARING
      Serial.println(F("Emergency released by operator"));
      resetCounters();
      setState(S_CLEARING);
    } else if (state == S_FAULT) {
      if (sensorsHealthy()) {
        Serial.println(F("Fault acknowledged by operator"));
        resetCounters();
        setState(S_CLEARING);
      } else {
        Serial.println(F("Fault NOT cleared: sensor still unhealthy"));
      }
    } else {
      enterSafe(S_EMERGENCY, F("EMERGENCY override pressed"));
    }
  }

  // ----- Sensor faults -----
  if (state != S_FAULT && state != S_EMERGENCY && !sensorsHealthy()) {
    enterSafe(S_FAULT, F("FAULT: sensor open-circuit or stuck"));
  }

  // ----- Axle events -----
  if (state != S_FAULT && state != S_EMERGENCY) {
    if (inRise)  onEntryAxle(now);
    if (outRise) onExitAxle(now);
  }

  // ----- FSM -----
  switch (state) {

    case S_CLEAR:
      setRoad(ROAD_GREEN);
      flash = false;
      buzz = BZ_OFF;
      barrierTarget = BARRIER_OPEN;
      break;

    case S_APPROACH:
      setRoad(ROAD_GREEN);
      flash = false;
      buzz = BZ_OFF;
      barrierTarget = BARRIER_OPEN;
      if ((long)(now - warnAt) >= 0) setState(S_WARNING);
      break;

    case S_WARNING:
      setRoad(ROAD_YELLOW);
      flash = true;
      buzz = BZ_WARN;
      barrierTarget = BARRIER_OPEN;
      if (now - tState >= yellowMs) setState(S_CLOSING);
      break;

    case S_CLOSING:
      setRoad(ROAD_RED);
      flash = true;
      buzz = BZ_WARN;
      barrierTarget = BARRIER_CLOSED;
      if (barrierPos == BARRIER_CLOSED) setState(S_OCCUPIED);
      break;

    case S_OCCUPIED:
      setRoad(ROAD_RED);
      flash = true;
      barrierTarget = BARRIER_CLOSED;
      if (!stalled && now - tState > OCCUPIED_TIMEOUT_MS) {
        stalled = true;
        Serial.println(F("!! STALLED TRAIN: crossing occupied too long"));
      }
      buzz = stalled ? BZ_STALLED : BZ_WARN;
      if (balanced()) setState(S_EXITING);
      break;

    case S_EXITING:
      setRoad(ROAD_RED);
      flash = true;
      buzz = BZ_WARN;
      barrierTarget = BARRIER_CLOSED;
      if (!balanced()) setState(S_OCCUPIED);              // more axles entered
      else if (now - lastExitMs >= EXIT_HOLD_MS) setState(S_CLEARING);
      break;

    case S_CLEARING:
      setRoad(ROAD_RED);                                  // stays red until barrier is up
      flash = true;
      buzz = BZ_OFF;
      barrierTarget = BARRIER_OPEN;
      if (barrierPos == BARRIER_OPEN) {
        if (axlesIn > 0) logTrain(now);
        yellowMs = YELLOW_MS;
        stepMs = BARRIER_STEP_MS;
        resetCounters();
        setRoad(ROAD_GREEN);
        flash = false;
        setState(S_CLEAR);
      }
      break;

    case S_FAULT:
    case S_EMERGENCY:
      flash = true;
      buzz = (state == S_EMERGENCY) ? BZ_EMERG : BZ_FAULT;
      if (safeYellow) {
        setRoad(ROAD_YELLOW);                             // no abrupt green -> red
        if (now - tState >= YELLOW_MS) safeYellow = false;
      } else {
        setRoad(ROAD_RED);
        barrierTarget = BARRIER_CLOSED;
      }
      break;
  }

  updateBarrier(now);
  updateFlashers(now);
  updateBuzzer(now);
  printStatus(now);
}
