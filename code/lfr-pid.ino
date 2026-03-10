// ================= MOTOR DRIVER =================
#define AIN1 7
#define AIN2 6
#define PWMA 5
#define BIN1 9
#define BIN2 10
#define PWMB 11
#define STBY 8

// ================= SENSOR =================
#define IR_ENABLE 4

#define LINE_THRESHOLD   500
#define WHITE_THRESHOLD  800

// ================= FINISH LED =================
#define FINISH_LED 13

// ================= CALIBRATION =================
int minVal[8] = {324, 318, 293, 291, 287, 244, 357, 325};
int maxVal[8] = {956, 953, 939, 934, 929, 943, 952, 956};
int sensor[8];

float weight[8] = {-3, -2, -1, -0.5, 0.5, 1, 2, 3};

// ================= PID =================
float Kp = 25;
float Kd = 12;
int baseSpeed = 120;
float lastError = 0;

// ================= STATES =================
enum State {
  FOLLOW_LINE,
  TURN_LEFT_STATE,
  TURN_RIGHT_STATE,
  UTURN_STATE,
  FINISH_STATE
};

State robotState = FOLLOW_LINE;

// ================= FLAGS & TIMERS =================
bool turnLocked = false;
unsigned long deadEndTimer = 0;
unsigned long uTurnStartTime = 0;
unsigned long finishTimer = 0;

// =================================================
void setup() {

  pinMode(IR_ENABLE, OUTPUT);
  digitalWrite(IR_ENABLE, HIGH);

  pinMode(FINISH_LED, OUTPUT);
  digitalWrite(FINISH_LED, LOW);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// =================================================
void loop() {

  readSensors();

  // ===== WHITE FINISH CHECK (TOP PRIORITY) =====
  if (robotState == FOLLOW_LINE) {
    if (isWhiteFinishBox()) {
      robotState = FINISH_STATE;
    }
  }

  bool leftA, straightA, rightA;
  detectJunctions(leftA, straightA, rightA);

  bool whiteCandidate = allSensorsWhite();

  switch (robotState) {

    // ================= FOLLOW LINE =================
    case FOLLOW_LINE:

      // ---------- DEAD END (BLOCKED IF WHITE) ----------
      if (!whiteCandidate && !leftA && !straightA && !rightA) {

        if (deadEndTimer == 0)
          deadEndTimer = millis();

        if (millis() - deadEndTimer > 120) {
          robotState = UTURN_STATE;
          turnLocked = true;
          lastError = 0;
          uTurnStartTime = 0;
          deadEndTimer = 0;
        }
        break;
      }
      else {
        deadEndTimer = 0;
      }

      // ---------- PRIORITY ----------
      if (!turnLocked && leftA) {
        slowDown();
        turnLocked = true;
        robotState = TURN_LEFT_STATE;
        lastError = 0;
      }
      else if (!turnLocked && straightA) {
        followLinePID();
      }
      else if (!turnLocked && rightA) {
        slowDown();
        turnLocked = true;
        robotState = TURN_RIGHT_STATE;
        lastError = 0;
      }
      break;

    // ================= TURN LEFT =================
    case TURN_LEFT_STATE:
      turnLeft();
      readSensors();
      if (lineFound()) {
        robotState = FOLLOW_LINE;
        turnLocked = false;
        lastError = 0;
      }
      break;

    // ================= TURN RIGHT =================
    case TURN_RIGHT_STATE:
      turnRight();
      readSensors();
      if (lineFound()) {
        robotState = FOLLOW_LINE;
        turnLocked = false;
        lastError = 0;
      }
      break;

    // ================= U TURN =================
    case UTURN_STATE:

      if (uTurnStartTime == 0)
        uTurnStartTime = millis();

      uTurn();
      readSensors();

      if ((millis() - uTurnStartTime > 200) && lineFound()) {
        robotState = FOLLOW_LINE;
        turnLocked = false;
        lastError = 0;
        uTurnStartTime = 0;
      }

      if (millis() - uTurnStartTime > 700) {
        robotState = FOLLOW_LINE;
        turnLocked = false;
        uTurnStartTime = 0;
      }
      break;

    // ================= FINISH =================
    case FINISH_STATE:
      stopMotors();
      digitalWrite(FINISH_LED, HIGH);
      while (1);
  }
}

// ============ WHITE FINISH LOGIC =================

bool isWhiteFinishBox() {

  if (!allSensorsWhite()) {
    finishTimer = 0;
    return false;
  }

  if (finishTimer == 0)
    finishTimer = millis();

  if (millis() - finishTimer > 600)
    return true;

  return false;
}

bool allSensorsWhite() {
  for (int i = 0; i < 8; i++) {
    if (sensor[i] < WHITE_THRESHOLD)
      return false;
  }
  return true;
}

// ================= PID FOLLOW ====================

void followLinePID() {

  float sum = 0, total = 0;

  for (int i = 0; i < 8; i++) {
    int raw = analogRead(A0 + i);
    int val = map(raw, minVal[i], maxVal[i], 0, 1000);
    val = constrain(val, 0, 1000);

    sum += val * weight[i];
    total += val;
  }

  float error = (total != 0) ? sum / total : lastError;

  float correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

// ================= MOTION ========================

int turnSpeed() {
  if (baseSpeed <= 100) return 120;
  if (baseSpeed <= 120) return 160;
  return 180;
}

void slowDown() {
  analogWrite(PWMA, 70);
  analogWrite(PWMB, 70);
  delay(15);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, turnSpeed());
  analogWrite(PWMB, turnSpeed());
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, turnSpeed());
  analogWrite(PWMB, turnSpeed());
}

void uTurn() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, turnSpeed());
  analogWrite(PWMB, turnSpeed());
}

// ================= SENSOR ========================

void readSensors() {
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(A0 + i);
    sensor[i] = map(raw, minVal[i], maxVal[i], 0, 1000);
    sensor[i] = constrain(sensor[i], 0, 1000);
  }
}

// ============ JUNCTION DETECTION =================

void detectJunctions(bool &leftA, bool &straightA, bool &rightA) {

  leftA  = (sensor[6] < LINE_THRESHOLD || sensor[7] < LINE_THRESHOLD);
  rightA = (sensor[0] < LINE_THRESHOLD || sensor[1] < LINE_THRESHOLD);

  int centerCount = 0;
  for (int i = 2; i <= 5; i++) {
    if (sensor[i] < LINE_THRESHOLD)
      centerCount++;
  }
  straightA = (centerCount >= 2);
}

// ============ LINE CONFIRMATION ==================

bool lineFound() {
  int count = 0;
  for (int i = 2; i <= 5; i++) {
    if (sensor[i] < LINE_THRESHOLD)
      count++;
  }
  return (count >= 2);
}
