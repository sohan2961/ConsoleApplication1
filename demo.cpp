#include <Servo.h>

// Motor Pins
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5

// IR Sensors
#define L_S A0
#define R_S A1

// Ultrasonic Sensor
#define trigPin 12
#define echoPin 13

// Color Sensor (TCS3200)
#define S0 11
#define S1 4
#define S2 3
#define S3 2
#define colorOut A2

// Voltage Monitoring
#define VOLTAGE_PIN A3
#define VOLTAGE_DIVIDER_RATIO 2.0 // Adjust based on your voltage divider
#define NOMINAL_VOLTAGE 7.4 // Your battery's nominal voltage

// Servo Motor
#define servoPin A5
Servo scanServo;
int servoPos = 50;                    // Current servo angle
bool sweepingRight = true;           // Direction toggle
unsigned long lastServoMoveTime = 0; // Time tracking

// Base speeds (will be adjusted based on voltage)
int baseForwardSpeed = 70;
int baseTurnSpeed = 50;
int currentForwardSpeed = baseForwardSpeed;
int currentTurnSpeed = baseTurnSpeed;

int approachThreshold = 15;  // cm
int scanThreshold = 2.8;    // cm

bool ultrasonicEnabled = true;
bool approaching = false;

// System parameters structure
typedef struct {
  float speedFactor;
  int forwardSpeed;
  int turnSpeed;
  int approachThreshold;
} SystemParams;

SystemParams currentParams;

void setup() {
  Serial.begin(9600);
  
  // Motor control pins
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  
  // Sensor pins
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  
  // Servo
  scanServo.attach(servoPin);
  scanServo.write(servoPos);
  
  // Initialize parameters
  updateSystemParameters();
}

void loop() {
  // Update system parameters based on current voltage
  updateSystemParameters();
  
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  // Servo scanning in range 40° ↔ 60°
  if (millis() - lastServoMoveTime >= 100) {
    if (sweepingRight) {
      servoPos += 20;
      if (servoPos >= 60) sweepingRight = false;
    } else {
      servoPos -= 20;
      if (servoPos <= 40) sweepingRight = true;
    }
    scanServo.write(servoPos);
    lastServoMoveTime = millis();
  }

  // Line following logic
  if (leftIR == LOW && rightIR == LOW) {
    forward();
  } else if (leftIR == HIGH && rightIR == LOW) {
    turnLeft();
  } else if (leftIR == LOW && rightIR == HIGH) {
    turnRight();
  } else {
    Stop();
  }

  // Obstacle detection
  if (distance <= currentParams.approachThreshold && !approaching) {
    approaching = true;
    Stop();
    delay(300);
    String color = getColor();
    handleDetectedColor(color);
    approaching = false;
  }
}

// Voltage monitoring functions
float readBatteryVoltage() {
  int sensorValue = analogRead(VOLTAGE_PIN);
  float voltage = sensorValue * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

float getVoltageCompensationFactor() {
  float currentVoltage = readBatteryVoltage();
  // Ensure we don't divide by zero or get extreme values
  currentVoltage = constrain(currentVoltage, 6.0, 8.4);
  return NOMINAL_VOLTAGE / currentVoltage;
}

void updateSystemParameters() {
  float compensation = getVoltageCompensationFactor();
  
  // Adjust speeds based on voltage
  currentParams.speedFactor = compensation;
  currentParams.forwardSpeed = baseForwardSpeed * compensation;
  currentParams.turnSpeed = baseTurnSpeed * compensation;
  
  // Constrain the speeds to safe limits
  currentParams.forwardSpeed = constrain(currentParams.forwardSpeed, 50, 255);
  currentParams.turnSpeed = constrain(currentParams.turnSpeed, 40, 200);
  
  // Update approach threshold if needed (could make it more sensitive at lower voltages)
  currentParams.approachThreshold = approachThreshold;
  
  // Update global speed variables
  currentForwardSpeed = currentParams.forwardSpeed;
  currentTurnSpeed = currentParams.turnSpeed;
}

// Modified motor control functions with voltage compensation
void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  analogWrite(enA, currentForwardSpeed);
  analogWrite(enB, currentForwardSpeed);
}

void backward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, currentForwardSpeed);
  analogWrite(enB, currentForwardSpeed);
}

void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, currentTurnSpeed);
  analogWrite(enB, currentTurnSpeed);
}

void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  analogWrite(enA, currentTurnSpeed);
  analogWrite(enB, currentTurnSpeed);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// Modified movement functions to use current speeds
void driveCurveLeft(int cm) {
  int t = cm * 30;
  analogWrite(enA, currentForwardSpeed * 0.6);
  analogWrite(enB, currentForwardSpeed);
  forward();
  delay(t);
  Stop();
}

void rotate180() {
  analogWrite(enA, currentForwardSpeed);
  analogWrite(enB, currentForwardSpeed);
  turnLeft();
  delay(800); // Calibrated for 180°
  Stop();
  delay(200);
}

// Existing color handler and other functions remain the same...
// Just replace any hardcoded speed values with currentForwardSpeed or currentTurnSpeed