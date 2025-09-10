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
volatile long currentPosition = 0;
volatile long targetPosition = 0;
volatile bool runallowed = false;
volatile bool homingMode = false;
volatile int directionMultiplier = 1;

// Yaw
long receivedYawSteps = 0;
long receivedYawSpeed = 0;
volatile long currentYawPosition = 0;
volatile long targetYawPosition = 0;
volatile bool runYawAllowed = false;
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
const float pitchRadius = 220.0;
const float yawRadius = 250.0;
const float stepsPerMM = 200.0;

// Flags
bool pitchCommandReady = false;
bool yawCommandReady = false;

// === Servo Motor ===
Servo myServo;
float servoAngle = 90.0;
bool servoIncreasing = false;
bool isServoMoving = false;
unsigned long lastServoTime = 0;
const int servoDelay = 20; // ms

void setup() {
  Serial.begin(9600);

  // Stepper pins
  pinMode(STEP_PIN_PITCH, OUTPUT);
  pinMode(DIR_PIN_PITCH, OUTPUT);
  pinMode(STEP_PIN_YAW, OUTPUT);
  pinMode(DIR_PIN_YAW, OUTPUT);

  // Servo setup
  myServo.attach(9);
  myServo.write((int)servoAngle);

  Serial.println("Stepper + Servo Control System Ready.");
  PrintCommands();
}

void loop() {
  checkSerial();
  RunPitchMotor();
  RunYawMotor();
  RunServoMotor();
}

// ========================= Pitch Motor =========================
void RunPitchMotor() {
  if (runallowed) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
      lastStepTime = now;
      digitalWrite(STEP_PIN_PITCH, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN_PITCH, LOW);
      currentPosition += (directionMultiplier == 1) ? 1 : -1;
      if (!homingMode && currentPosition == targetPosition)
        runallowed = false;
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
  digitalWrite(DIR_PIN_PITCH, directionMultiplier == 1 ? HIGH : LOW);
  stepInterval = 1000000UL / receivedSpeed;
}

void GoHomePitch() {
  runallowed = true;
  homingMode = true;
  targetPosition = 0;
  directionMultiplier = (currentPosition > 0) ? -1 : 1;
  digitalWrite(DIR_PIN_PITCH, directionMultiplier == 1 ? HIGH : LOW);
  stepInterval = 1000000UL / 400;
}

// ========================= Yaw Motor =========================
void RunYawMotor() {
  if (runYawAllowed) {
    unsigned long now = micros();
    if (now - lastYawStepTime >= yawStepInterval) {
      lastYawStepTime = now;
      digitalWrite(STEP_PIN_YAW, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN_YAW, LOW);
      currentYawPosition += (yawDirection == 1) ? 1 : -1;
      if (currentYawPosition == targetYawPosition)
        runYawAllowed = false;
    }
  }
}

void RotateYawRelative() {
  runYawAllowed = true;
  targetYawPosition = currentYawPosition + (yawDirection * receivedYawSteps);
  digitalWrite(DIR_PIN_YAW, yawDirection == 1 ? HIGH : LOW);
  yawStepInterval = 1000000UL / receivedYawSpeed;
}

// ========================= Servo Motor =========================
void RunServoMotor() {
  if (isServoMoving && millis() - lastServoTime >= servoDelay) {
    lastServoTime = millis();
    myServo.write((int)servoAngle);
    if (servoIncreasing) {
      servoAngle += 0.5;
      if (servoAngle >= 180.0) {
        servoAngle = 180.0;
        servoIncreasing = false;
      }
    } else {
      servoAngle -= 0.5;
      if (servoAngle <= 0.0) {
        servoAngle = 0.0;
        servoIncreasing = true;
      }
    }
  }
}

// ========================= Serial Handler =========================
void checkSerial() {
  if (Serial.available() > 0) {
    receivedCommand = Serial.read();
    newData = true;

    if (newData) {
      switch (receivedCommand) {
        case 'P':
        case 'N':
          {
            float angleDeg = Serial.parseFloat();
            receivedSpeed = Serial.parseFloat();
            float radians = angleDeg * PI / 180.0;
            float linearTravel = tan(radians) * pitchRadius;
            receivedSteps = long(linearTravel * stepsPerMM);
            directionMultiplier = (receivedCommand == 'P') ? 1 : -1;
            pitchCommandReady = true;

            Serial.print("Pitch Tilt: ");
            Serial.print(angleDeg); Serial.print("° → ");
            Serial.print(linearTravel, 2); Serial.print(" mm → ");
            Serial.print(receivedSteps); Serial.println(" steps");
          }
          break;

        case 'Y':
        case 'A':
          {
            float yawAngle = Serial.parseFloat();
            receivedYawSpeed = Serial.parseFloat();
            float arcLength = (yawAngle * PI * yawRadius) / 180.0;
            receivedYawSteps = long(arcLength * stepsPerMM);
            yawDirection = (receivedCommand == 'Y') ? 1 : -1;
            yawCommandReady = true;

            Serial.print("Yaw Rotate: ");
            Serial.print(yawAngle); Serial.print("° → ");
            Serial.print(arcLength, 2); Serial.print(" mm → ");
            Serial.print(receivedYawSteps); Serial.println(" steps");
          }
          break;

        case 'V': // Start servo
          isServoMoving = true;
          Serial.println("Servo started.");
          break;

        case 'S': // Stop everything
          runallowed = false;
          runYawAllowed = false;
          pitchCommandReady = false;
          yawCommandReady = false;
          isServoMoving = false;
          Serial.println("All motion stopped.");
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

  if (pitchCommandReady && yawCommandReady) {
    RotateRelativePitch();
    RotateYawRelative();
    pitchCommandReady = false;
    yawCommandReady = false;
    Serial.println("Started both Pitch and Yaw motors.");
  }
}

// ========================= Command Help =========================
void PrintCommands() {
  Serial.println("=== COMMAND LIST ===");
  Serial.println(" P <tilt_angle_deg> <speed> : Pitch Tilt CW (raise)");
  Serial.println(" N <tilt_angle_deg> <speed> : Pitch Tilt CCW (lower)");
  Serial.println(" Y <yaw_angle_deg> <speed>  : Yaw CW");
  Serial.println(" A <yaw_angle_deg> <speed>  : Yaw CCW");
  Serial.println(" V : Start sweeping servo on pin 9");
  Serial.println(" S : Stop all motion and servo");
  Serial.println(" H : Home pitch motor");
  Serial.println(" L : Show current position");
  Serial.println(" U : Reset position to zero");
  Serial.println(" C : Print command list");
}
