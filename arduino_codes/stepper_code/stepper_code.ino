#include <Servo.h>

// === Pin definitions ===
#define STEP_PIN_PITCH 3
#define DIR_PIN_PITCH  4
#define STEP_PIN_YAW   5
#define DIR_PIN_YAW    6

// === Motor State Variables ===
// Pitch
long receivedSteps = 0;
long receivedSpeed = 0;
volatile long currentPosition = -7520;
volatile long targetPosition = -7520;
volatile bool runallowed = false;
volatile bool homingModePitch = false;
volatile int directionMultiplier = 1;

// Yaw
long receivedYawSteps = 0;
long receivedYawSpeed = 0;
volatile long currentYawPosition = -204;
volatile long targetYawPosition = -204;
volatile bool runYawAllowed = false;
volatile bool homingModeYaw = false;
volatile int yawDirection = 1;

// Serial Input
char receivedCommand;
bool newData = false;

// Time control
unsigned long lastStepTime = 0;
unsigned long stepInterval = 1000;
unsigned long lastYawStepTime = 0;
unsigned long yawStepInterval = 1000;

// Constants
const float pitchPlatformRadius = 250.0; // mm
const float yawRadius = 350.0;           // mm
const float stepsPerMM = 200.0;
const float yawGearRadius = 38.0;
const float yawStepsPerRev = 200.0;

// Home Positions
const long pitchHomePosition = -7520;
const long yawHomePosition = -204;

// Flags
bool pitchCommandReady = false;
bool yawCommandReady = false;

// Servo
Servo myServo;
float angle = 90.0;
bool increasing = false;
bool isServoMoving = false;

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN_PITCH, OUTPUT);
  pinMode(DIR_PIN_PITCH, OUTPUT);
  pinMode(STEP_PIN_YAW, OUTPUT);
  pinMode(DIR_PIN_YAW, OUTPUT);

  myServo.attach(9);
  myServo.write((int)angle);

  Serial.println("Stepper Control: Pitch (by Tilt) & Yaw (by Angle)");
  PrintCommands();
}

void loop() {
  checkSerial();
  RunPitchMotor();
  RunYawMotor();
  SweepServo();
}

// ========================= Pitch Motor Section =========================

void RunPitchMotor() {
  if (runallowed) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
      lastStepTime = now;

      digitalWrite(STEP_PIN_PITCH, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN_PITCH, LOW);

      currentPosition += (directionMultiplier == 1) ? 1 : -1;

      if (homingModePitch) {
        if (currentPosition == pitchHomePosition) {
          runallowed = false;
          homingModePitch = false;
          Serial.println("Pitch Homed.");
        }
      } else {
        if (currentPosition == targetPosition) {
          runallowed = false;
        }
      }
    }
  }
}

void RotateRelativePitch() {
  runallowed = true;
  targetPosition = currentPosition + (directionMultiplier * receivedSteps);
  digitalWrite(DIR_PIN_PITCH, directionMultiplier == 1 ? HIGH : LOW);
  stepInterval = 1000000UL / receivedSpeed;
}

void GoHomePitch() {
  runallowed = true;
  homingModePitch = true;
  targetPosition = pitchHomePosition;
  directionMultiplier = (currentPosition < pitchHomePosition) ? 1 : -1;
  digitalWrite(DIR_PIN_PITCH, directionMultiplier == 1 ? HIGH : LOW);
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

      if (homingModeYaw) {
        if (currentYawPosition == yawHomePosition) {
          runYawAllowed = false;
          homingModeYaw = false;
          Serial.println("Yaw Homed.");
        }
      } else {
        if (currentYawPosition == targetYawPosition) {
          runYawAllowed = false;
        }
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

void GoHomeYaw() {
  runYawAllowed = true;
  homingModeYaw = true;
  targetYawPosition = yawHomePosition;
  yawDirection = (currentYawPosition < yawHomePosition) ? 1 : -1;
  digitalWrite(DIR_PIN_YAW, yawDirection == 1 ? HIGH : LOW);
  yawStepInterval = 1000000UL / 400;
}

// ========================= Serial Input Handler =========================

void checkSerial() {
  if (Serial.available() > 0) {
    receivedCommand = Serial.read();
    newData = true;

    if (newData) {
      switch (receivedCommand) {
        case 'P':
        case 'N': {
          float angleDeg = Serial.parseFloat();
          receivedSpeed = Serial.parseFloat();

          float radians = angleDeg * PI / 180.0;
          float linearTravel = 0.5 * tan(radians) * pitchPlatformRadius;
          receivedSteps = long(linearTravel * stepsPerMM);

          directionMultiplier = (receivedCommand == 'P') ? 1 : -1;
          pitchCommandReady = true;

          Serial.print("Pitch Tilt Command: ");
          Serial.print(angleDeg); Serial.print("° | Travel: ");
          Serial.print(linearTravel, 2); Serial.print(" mm | Steps: ");
          Serial.println(receivedSteps);
          break;
        }

        case 'Y':
        case 'A': {
          float yawAngle = Serial.parseFloat();
          receivedYawSpeed = Serial.parseFloat();

          float arcLength = (yawAngle * PI * yawRadius) / 180.0;
          float gearCircumference = 2 * PI * yawGearRadius;
          float revolutions = arcLength / gearCircumference;
          receivedYawSteps = long(2 * revolutions * yawStepsPerRev);

          yawDirection = (receivedCommand == 'Y') ? 1 : -1;
          yawCommandReady = true;

          Serial.print("Yaw Rotate: ");
          Serial.print(yawAngle); Serial.print("° → ");
          Serial.print(arcLength, 2); Serial.print(" mm → ");
          Serial.print(receivedYawSteps); Serial.println(" steps");
          break;
        }

        case 'V':
          isServoMoving = true;
          Serial.println("Servo started sweeping.");
          break;

        case 'S':
          runallowed = false;
          runYawAllowed = false;
          pitchCommandReady = false;
          yawCommandReady = false;
          isServoMoving = false;
          Serial.println("Motors & Servo Stopped.");
          break;

        case 'H':
          GoHomePitch();
          GoHomeYaw();
          Serial.println("Homing Pitch and Yaw...");
          break;

        case 'U':
          currentPosition = pitchHomePosition;
          targetPosition = pitchHomePosition;
          currentYawPosition = yawHomePosition;
          targetYawPosition = yawHomePosition;
          Serial.println("Position Reset to Home.");
          break;

        case 'L':
          Serial.print("Pitch Pos: "); Serial.print(currentPosition);
          Serial.print(" | Yaw Pos: "); Serial.println(currentYawPosition);
          break;

        case 'C':
          PrintCommands();
          break;
      }

      newData = false;
    }
  }

  if (pitchCommandReady && yawCommandReady) {
    RotateRelativePitch();
    RotateYawRelative();
    pitchCommandReady = false;
    yawCommandReady = false;
    Serial.println("Started both Pitch and Yaw motors.");
  }
}

// ========================= Servo Sweeping =========================

void SweepServo() {
  if (isServoMoving) {
    myServo.write((int)angle);
    delay(20);

    if (increasing) {
      angle += 0.5;
      if (angle >= 180.0) {
        angle = 180.0;
        increasing = false;
      }
    } else {
      angle -= 0.5;
      if (angle <= 0.0) {
        angle = 0.0;
        increasing = true;
      }
    }
  }
}

// ========================= Command Help =========================

void PrintCommands() {
  Serial.println("=== COMMAND LIST ===");
  Serial.println(" P <tilt_angle_deg> <speed> : Pitch Tilt CW (raise)");
  Serial.println(" N <tilt_angle_deg> <speed> : Pitch Tilt CCW (lower)");
  Serial.println(" Y <yaw_angle_deg> <speed>  : Yaw CW");
  Serial.println(" A <yaw_angle_deg> <speed>  : Yaw CCW");
  Serial.println(" V : Start servo sweep");
  Serial.println(" S : Stop all motion (stepper & servo)");
  Serial.println(" H : Home pitch and yaw motors");
  Serial.println(" U : Reset current positions to home");
  Serial.println(" L : Show current positions");
  Serial.println(" C : Print command list");
}













