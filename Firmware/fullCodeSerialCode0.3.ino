#include <AccelStepper.h>
#include <string.h>  // for strtok, strcasecmp

// ───── CONFIGURATION ─────────────────────────────────────────────
const uint8_t N_MOTORS = 6;
const long    SLOW_ZONE = 10;  // steps within target to switch speeds
// Steps per full 360° rotation for each motor (adjust for microstepping)
const int stepsPerRevolution[N_MOTORS] = {200, 200, 200, 200, 200, 200};

// Motion parameters (editable at top)
long accelerations[N_MOTORS] = {1000, 1000, 1000, 1000, 1000, 1000};
float fastSpeeds[N_MOTORS]   = {1000, 1000, 1000, 1000, 1000, 1000};
float slowSpeeds[N_MOTORS]   = { 700,  700,  700,  700,  700,  700};

// Pin assignments: DIR and STEP (or PUL) for each motor
const uint8_t DIR_PINS[N_MOTORS]  = { 2, 12,  8,  6,  8, 10 };
const uint8_t STEP_PINS[N_MOTORS] = { 3, 11,  4,  7,  9,  1 };

AccelStepper* steppers[N_MOTORS];

// ── Batch-tracking flags ─────────────────────────────────────────
bool pendingMotion = false;  // Was there any P command in the last batch?
bool doneSent       = true;  // Have we already emitted DONE for the last batch?
// ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) ; // wait for Serial

  Serial.println(F("--- Stepper Control Ready ---"));
  Serial.println(F("Send multiple commands in one line, e.g.:"));
  Serial.println(F("  P 2 200 P 1 200 P 0 -200"));

  // Instantiate and configure each stepper
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    steppers[i] = new AccelStepper(AccelStepper::DRIVER,
                                   STEP_PINS[i],
                                   DIR_PINS[i]);
    steppers[i]->setAcceleration(accelerations[i]);
    steppers[i]->setMaxSpeed(fastSpeeds[i]);
    // Invert direction for motor index 2 if needed
    if (i == 2) {
      steppers[i]->setPinsInverted(true, false);
    }
  }
}

void loop() {
  handleSerial();

  // Adjust speeds based on remaining distance
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    long d = steppers[i]->distanceToGo();
    steppers[i]->setMaxSpeed(
      abs(d) > SLOW_ZONE ? fastSpeeds[i] : slowSpeeds[i]
    );
  }

  // Run all steppers (they move in parallel)
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    steppers[i]->run();
  }

  // ── Deferred DONE: once motion finishes, send DONE exactly once ──
  if (!doneSent) {
    bool allStopped = true;
    for (uint8_t i = 0; i < N_MOTORS; i++) {
      if (steppers[i]->distanceToGo() != 0) {
        allStopped = false;
        break;
      }
    }
    if (allStopped) {
      Serial.println("DONE");
      doneSent = true;
    }
  }
}

// ───── Serial‐command parser (multi‐command enabled) ───────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // Mark start of a new batch
  pendingMotion = false;

  // Copy into modifiable C-string
  size_t bufSize = line.length() + 1;
  char buf[bufSize];
  line.toCharArray(buf, bufSize);

  // Walk tokens
  char* token = strtok(buf, " ");
  while (token) {
    // Acceleration: A idx val
    if (strcasecmp(token, "A") == 0) {
      char* sIdx = strtok(nullptr, " ");
      char* sVal = strtok(nullptr, " ");
      if (sIdx && sVal) {
        int idx = atoi(sIdx);
        float val = atof(sVal);
        if (checkIdx(idx)) {
          accelerations[idx] = val;
          steppers[idx]->setAcceleration(val);
          Serial.print("Motor ");
          Serial.print(idx);
          Serial.print(" acceleration=");
          Serial.println(val);
        }
      } else {
        Serial.println("Malformed A command");
        break;
      }
    }
    // Fast speed: T idx val
    else if (strcasecmp(token, "T") == 0) {
      char* sIdx = strtok(nullptr, " ");
      char* sVal = strtok(nullptr, " ");
      if (sIdx && sVal) {
        int idx = atoi(sIdx);
        float val = atof(sVal);
        if (checkIdx(idx)) {
          fastSpeeds[idx] = val;
          Serial.print("Motor ");
          Serial.print(idx);
          Serial.print(" fastSpeed=");
          Serial.println(val);
        }
      } else {
        Serial.println("Malformed T command");
        break;
      }
    }
    // Slow speed: L idx val
    else if (strcasecmp(token, "L") == 0) {
      char* sIdx = strtok(nullptr, " ");
      char* sVal = strtok(nullptr, " ");
      if (sIdx && sVal) {
        int idx = atoi(sIdx);
        float val = atof(sVal);
        if (checkIdx(idx)) {
          slowSpeeds[idx] = val;
          Serial.print("Motor ");
          Serial.print(idx);
          Serial.print(" slowSpeed=");
          Serial.println(val);
        }
      } else {
        Serial.println("Malformed L command");
        break;
      }
    }
    // Position: P idx deg
    else if (strcasecmp(token, "P") == 0) {
      char* sIdx = strtok(nullptr, " ");
      char* sDeg = strtok(nullptr, " ");
      if (sIdx && sDeg) {
        int idx = atoi(sIdx);
        float deg = atof(sDeg);
        if (checkIdx(idx)) {
          long steps = lround(deg / 360.0 * stepsPerRevolution[idx]);
          steppers[idx]->moveTo(steps);
          pendingMotion = true;
          Serial.print("Motor ");
          Serial.print(idx);
          Serial.print(" target=");
          Serial.print(deg, 1);
          Serial.print(F("° ("));
          Serial.print(steps);
          Serial.println(F(" steps)"));
        }
      } else {
        Serial.println("Malformed P command");
        break;
      }
    }
    else {
      Serial.print("Unknown token: ");
      Serial.println(token);
      break;
    }

    token = strtok(nullptr, " ");
  }

  // ── Immediate vs. deferred DONE ────────────────────────────────
  if (!pendingMotion) {
    // No moves in this batch → acknowledge now
    Serial.println("DONE");
    doneSent = true;
  } else {
    // There is motion → defer DONE until motors finish
    doneSent = false;
  }
}

bool checkIdx(int i) {
  if (i < 0 || i >= N_MOTORS) {
    Serial.println(F("Index out of range"));
    return false;
  }
  return true;
}
