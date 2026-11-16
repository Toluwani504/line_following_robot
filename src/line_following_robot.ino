/*
  LINE FOLLOWER ROBOT - PID control with 2 digital IR sensors
  Board : Arduino Uno          Driver : L298N          Motors : 2 x TT gear motor

  PIN MAP (same as your wiring diagram)
    IR sensor LEFT  OUT -> D12        IR sensor RIGHT OUT -> D11
    RIGHT motor : ENA/EN = D6 (PWM), IN1 = D7, IN2 = D8
    LEFT  motor : ENB/EN = D5 (PWM), IN3 = D9, IN4 = D10
    D13 = on-board LED (lights when a sensor sees the black line)
*/

// ---------------- PINS ----------------
const byte IR_SENSOR_RIGHT = 11;
const byte IR_SENSOR_LEFT  = 12;

const byte enableRightMotor = 6;
const byte rightMotorPin1   = 7;
const byte rightMotorPin2   = 8;

const byte enableLeftMotor  = 5;
const byte leftMotorPin1    = 9;
const byte leftMotorPin2    = 10;

const byte STATUS_LED       = 13;

// ---------------- SWITCHES YOU CAN CHANGE ----------------
const bool LINE_IS_HIGH       = true;   // true: sensor output is HIGH over the black line
const bool SWAP_SENSORS       = false;  // true: if the robot turns AWAY from the line
const bool INVERT_LEFT_MOTOR  = false;  // true: if the left wheel spins backwards
const bool INVERT_RIGHT_MOTOR = false;  // true: if the right wheel spins backwards
const bool STOP_ON_BOTH       = true;   // stop when both sensors see black (finish line / lifted)

const bool SENSOR_TEST_MODE   = false;  // true: motors OFF, prints sensor states (for calibration)
const bool MOTOR_TEST_AT_START= false;  // true: spins each wheel forward once at start-up
const bool DEBUG_PRINT        = false;  // true: prints error/correction to Serial Monitor

// ---------------- SPEED SETTINGS (percent) ----------------
const int BASE_SPEED   = 55;   // normal forward speed, 0-100. Start low, raise later.
const int MAX_REVERSE  = 40;   // the inner wheel may reverse up to this much in sharp turns
const int MOTOR_BALANCE = 0;   // + makes LEFT motor faster, - makes RIGHT motor faster
                               // (use it if the robot drifts on a straight line)

// TT motors do not turn at very low PWM, so every non-zero speed is lifted above MIN_PWM.
const int MIN_PWM = 90;        // lowest PWM at which your wheels actually turn
const int MAX_PWM = 230;       // highest PWM used (protects TT motors)

// ---------------- PID GAINS (tune these) ----------------
float Kp = 24.0;   // steering strength when a sensor sees the line
float Ki = 0.35;   // extra steering the longer the line stays on one side
float Kd = 18.0;   // brakes the swing when the line returns to the middle

const float I_MAX     = 50.0;  // limit of the accumulated error
const float I_RELEASE = 0.80;  // how fast the accumulated error is released at the centre
const float D_DECAY   = 0.90;  // how fast the derivative kick fades

// ---------------- TIMING ----------------
// The PWM speed-up in setup() makes Timer0 run 8x faster, so micros() and delay()
// run 8x too fast. T0 corrects for it. Do not use plain delay() in this sketch.
const unsigned long T0       = 8UL;
const unsigned long STEP_US  = 5000UL * T0;  // one control step = 5 ms of real time
const byte DEBOUNCE_STEPS    = 2;            // sensor must be stable for 2 steps (10 ms)
const byte BOTH_STOP_STEPS   = 12;           // both black for 60 ms before stopping
const byte RAMP_STEPS        = 80;           // 400 ms soft start (protects the supply)

// ---------------- STATE ----------------
unsigned long lastStep = 0;
bool  leftState = false, rightState = false;
byte  leftCount = 0,  rightCount = 0;
byte  bothSteps = 0;
byte  rampSteps = 0;
int   lastError = 0;
float integral  = 0;
float dTerm     = 0;
byte  printCount = 0;

void realDelay(unsigned long ms) { delay(ms * T0); }

bool isBlack(byte pin) {
  int v = digitalRead(pin);
  return LINE_IS_HIGH ? (v == HIGH) : (v == LOW);
}

// a sensor value is accepted only after it stays the same for DEBOUNCE_STEPS steps
void debounce(bool raw, bool &state, byte &count) {
  if (raw == state) { count = 0; }
  else if (++count >= DEBOUNCE_STEPS) { state = raw; count = 0; }
}

int pctToPwm(int pct) {
  if (pct == 0) return 0;
  int mag = abs(pct);
  if (mag > 100) mag = 100;
  return MIN_PWM + (int)((long)(MAX_PWM - MIN_PWM) * mag / 100);
}

void driveMotor(byte en, byte in1, byte in2, int pct, bool invert) {
  if (abs(pct) < 2) pct = 0;
  if (invert) pct = -pct;
  if (pct > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (pct < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  analogWrite(en, pctToPwm(pct));
}

void setMotors(int leftPct, int rightPct) {
  driveMotor(enableLeftMotor,  leftMotorPin1,  leftMotorPin2,  leftPct,  INVERT_LEFT_MOTOR);
  driveMotor(enableRightMotor, rightMotorPin1, rightMotorPin2, rightPct, INVERT_RIGHT_MOTOR);
}

void stopMotors() { setMotors(0, 0); }

void resetController() {
  integral = 0; dTerm = 0; lastError = 0; rampSteps = 0;
}

void setup() {
  // faster PWM on pins 5 and 6 (7.8 kHz) so TT motors run smoothly at low speed
  TCCR0B = (TCCR0B & B11111000) | B00000010;

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(STATUS_LED, OUTPUT);

  Serial.begin(9600);
  stopMotors();
  Serial.println(F("BOOT"));

  if (MOTOR_TEST_AT_START && !SENSOR_TEST_MODE) {
    realDelay(2000);
    setMotors(50, 0);  realDelay(800);   // LEFT wheel forward
    stopMotors();      realDelay(500);
    setMotors(0, 50);  realDelay(800);   // RIGHT wheel forward
    stopMotors();
  }
  realDelay(2000);                        // time to put the robot on the track
  lastStep = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - lastStep < STEP_US) return;   // run exactly one control step every 5 ms
  lastStep = now;

  // ---- 1. read and clean the sensors ----
  bool rawL = isBlack(SWAP_SENSORS ? IR_SENSOR_RIGHT : IR_SENSOR_LEFT);
  bool rawR = isBlack(SWAP_SENSORS ? IR_SENSOR_LEFT  : IR_SENSOR_RIGHT);
  debounce(rawL, leftState,  leftCount);
  debounce(rawR, rightState, rightCount);
  bool leftBlack = leftState, rightBlack = rightState;
  digitalWrite(STATUS_LED, (leftBlack || rightBlack) ? HIGH : LOW);

  if (SENSOR_TEST_MODE) {
    stopMotors();
    if (++printCount >= 20) {
      printCount = 0;
      Serial.print(F("LEFT: "));  Serial.print(leftBlack ? F("BLACK") : F("white"));
      Serial.print(F("   RIGHT: ")); Serial.println(rightBlack ? F("BLACK") : F("white"));
    }
    return;
  }

  // ---- 2. work out the error: -1 line on left, 0 centred, +1 line on right ----
  int error;
  if (leftBlack && rightBlack) {
    bothSteps++;
    if (STOP_ON_BOTH && bothSteps >= BOTH_STOP_STEPS) {   // really both black: stop
      stopMotors();
      resetController();
      return;
    }
    error = STOP_ON_BOTH ? lastError : 0;   // short overlap: keep the last decision
  } else {
    bothSteps = 0;
    error = rightBlack ? 1 : (leftBlack ? -1 : 0);
  }

  // ---- 3. PID ----
  if (error == 0) {
    integral *= I_RELEASE;                             // line is centred: release
  } else {
    if ((error > 0 && integral < 0) || (error < 0 && integral > 0)) integral = 0;
    integral += error;                                 // stays on one side: build up
    if (integral >  I_MAX) integral =  I_MAX;
    if (integral < -I_MAX) integral = -I_MAX;
  }
  dTerm = dTerm * D_DECAY + (float)(error - lastError);
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * dTerm;

  // ---- 4. soft start, then motor speeds ----
  if (rampSteps < RAMP_STEPS) rampSteps++;
  float base = (float)BASE_SPEED * rampSteps / RAMP_STEPS;

  float l = base + correction + MOTOR_BALANCE;   // + correction = turn right
  float r = base - correction - MOTOR_BALANCE;
  int leftPct  = (int)(l >= 0 ? l + 0.5 : l - 0.5);
  int rightPct = (int)(r >= 0 ? r + 0.5 : r - 0.5);
  leftPct  = constrain(leftPct,  -MAX_REVERSE, 100);
  rightPct = constrain(rightPct, -MAX_REVERSE, 100);
  setMotors(leftPct, rightPct);

  if (DEBUG_PRINT && ++printCount >= 20) {
    printCount = 0;
    Serial.print(F("err ")); Serial.print(error);
    Serial.print(F("  corr ")); Serial.print(correction);
    Serial.print(F("  L ")); Serial.print(leftPct);
    Serial.print(F("  R ")); Serial.println(rightPct);
  }
}
