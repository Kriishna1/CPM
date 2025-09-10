// === Pin definitions for Pitch Motors ===
#define STEP_PIN_PITCH1 3
#define DIR_PIN_PITCH1  4
#define STEP_PIN_PITCH2 5
#define DIR_PIN_PITCH2  6

// === Pin definitions for Yaw Motor ===
#define STEP_PIN_YAW 7
#define DIR_PIN_YAW  8

// === Global variables ===
// Pitch Motor
long receivedSteps = 0;
long receivedSpeed = 0;
volatile long currentPosition = 0;
volatile long targetPosition = 0;
volatile bool runallowed = false;
volatile bool homingMode = false;
volatile int directionMultiplier = 1;

// Yaw Motor
long receivedYawSteps = 0;
long receivedYawSpeed = 0;
volatile long currentYawPosition = 0;
volatile long targetYawPosition = 0;
volatile bool runYawAllowed = false;
volatile int yawDirection = 1;

char receivedCommand;
bool newData = false;

unsigned long lastStepTime = 0;
unsigned long stepInterval = 1000;
unsigned long lastYawStepTime = 0;
unsigned long yawStepInterval = 1000;

void setup() {
  Serial.begin(9600);

  pinMode(STEP_PIN_PITCH1, OUTPUT);
  pinMode(DIR_PIN_PITCH1, OUTPUT);
  pinMode(STEP_PIN_PITCH2, OUTPUT);
  pinMode(DIR_PIN_PITCH2, OUTPUT);
  pinMode(STEP_PIN_YAW, OUTPUT);
  pinMode(DIR_PIN_YAW, OUTPUT);

  Serial.println("Pitch & Yaw Motor Control");
  Serial.println("First enter pitch command, then yaw command.");
  Serial.println("Use 'C' to list commands.");
}

void loop() {
  checkSerial();
  RunPitchMotors();
  RunYawMotor();
}

// ========================= Pitch Motor Section =========================

void RunPitchMotors() {
  if (runallowed) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
      lastStepTime = now;

      digitalWrite(STEP_PIN_PITCH1, HIGH);
      digitalWrite(STEP_PIN_PITCH2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN_PITCH1, LOW);
      digitalWrite(STEP_PIN_PITCH2, LOW);

      currentPosition += (directionMultiplier == 1) ? 1 : -1;

      if (!homingMode && currentPosition == targetPosition) {
        runallowed = false;
      }
      if (homingMode && currentPosition == 0) {
        homingMode = false;
        runallowed = false;
        Serial.println("Pitch Homed.");
      }
    }
  }
}

void RotateRelativePitch() {
  runallowed = true;
  targetPosition = currentPosition + (directionMultiplier * receivedSteps);
  setPitchDirection(directionMultiplier);
  stepInterval = 1000000UL / receivedSpeed;
}

void RotateAbsolutePitch() {
  runallowed = true;
  targetPosition = directionMultiplier * receivedSteps;
  long delta = targetPosition - currentPosition;
  directionMultiplier = (delta >= 0) ? 1 : -1;
  setPitchDirection(directionMultiplier);
  stepInterval = 1000000UL / receivedSpeed;
}

void setPitchDirection(int multiplier) {
  digitalWrite(DIR_PIN_PITCH1, multiplier == 1 ? HIGH : LOW);
  digitalWrite(DIR_PIN_PITCH2, multiplier == 1 ? HIGH : LOW);
}

void GoHomePitch() {
  runallowed = true;
  homingMode = true;
  targetPosition = 0;
  directionMultiplier = (currentPosition > 0) ? -1 : 1;
  setPitchDirection(directionMultiplier);
  stepInterval = 1000000UL / 400;
}

// ========================= Yaw Motor Section =========================

void RunYawMotor() {
  if (runYawAllowed) {
    unsigned long now = micros();
    if (now - lastYawStepTime >= yawStepInterval) {
      lastYawStepTime = now;

      digitalWrite(STEP_PIN_YAW, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN_YAW, LOW);

      currentYawPosition += (yawDirection == 1) ? 1 : -1;

      if (currentYawPosition == targetYawPosition) {
        runYawAllowed = false;
      }
    }
  }
}

void RotateYawRelative() {
  runYawAllowed = true;
  targetYawPosition = currentYawPosition + (yawDirection * receivedYawSteps);
  digitalWrite(DIR_PIN_YAW, yawDirection == 1 ? HIGH : LOW);
  yawStepInterval = 1000000UL / receivedYawSpeed;
}

// ========================= Serial Communication =========================

void checkSerial() {
  if (Serial.available() > 0) {
    receivedCommand = Serial.read();
    newData = true;

    if (newData) {
      switch (receivedCommand) {
        // === Pitch Commands ===
        case 'P':
          receivedSteps = Serial.parseFloat();
          receivedSpeed = Serial.parseFloat();
          directionMultiplier = 1;
          RotateRelativePitch();
          Serial.println("Pitch Relative +");
          delay(200);  // Wait before asking yaw input
          getYawCommand();  // Ask for yaw
          break;

        case 'N':
          receivedSteps = Serial.parseFloat();
          receivedSpeed = Serial.parseFloat();
          directionMultiplier = -1;
          RotateRelativePitch();
          Serial.println("Pitch Relative -");
          delay(200);
          getYawCommand();
          break;

        case 'S':
          runallowed = false;
          runYawAllowed = false;
          Serial.println("Motors Stopped.");
          break;

        case 'H':
          GoHomePitch();
          break;

        case 'U':
          currentPosition = 0;
          targetPosition = 0;
          currentYawPosition = 0;
          targetYawPosition = 0;
          Serial.println("Position Reset.");
          break;

        case 'L':
          Serial.print("Pitch Pos: "); Serial.print(currentPosition);
          Serial.print(" | Yaw Pos: "); Serial.println(currentYawPosition);
          break;

        case 'C':
          PrintCommands();
          break;

        default:
          break;
      }
      newData = false;
    }
  }
}

void getYawCommand() {
  Serial.println("Enter Yaw command: Y <steps> <speed>");
  while (!Serial.available());
  if (Serial.read() == 'Y') {
    receivedYawSteps = Serial.parseFloat();
    receivedYawSpeed = Serial.parseFloat();
    yawDirection = (receivedYawSteps >= 0) ? 1 : -1;
    receivedYawSteps = abs(receivedYawSteps);
    RotateYawRelative();
    Serial.println("Yaw Command Executed.");
  }
}

void PrintCommands() {
  Serial.println("Commands:");
  Serial.println(" P <steps> <speed> : Pitch CW (relative), then prompt for Yaw");
  Serial.println(" N <steps> <speed> : Pitch CCW (relative), then prompt for Yaw");
  Serial.println(" Y <steps> <speed> : (After pitch) Move yaw motor");
  Serial.println(" H : Home pitch");
  Serial.println(" S : Stop both motors");
  Serial.println(" L : Print positions");
  Serial.println(" U : Reset positions to 0");
}