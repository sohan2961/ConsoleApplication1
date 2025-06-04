#include <Servo.h> 

#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A0
#define R_S A1
#define trigPin 12
#define echoPin 13
#define S0 11
#define S1 4
#define S2 3
#define S3 2
#define colorOut A2
#define servoPin A5
#define voltagePin A3  // Added for voltage monitoring

Servo scanServo;
int servoPos = 50;                    
bool sweepingRight = true;           
unsigned long lastServoMoveTime = 0; 

// Base parameters (will be adjusted based on voltage)
int baseForwardSpeed = 70;
int baseTurnSpeed = 50;
int approachThreshold = 15;  // cm
int scanThreshold = 2.8;     // cm

// Current parameters (will be adjusted)
int forwardSpeed = baseForwardSpeed;
int turnSpeed = baseTurnSpeed;

bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

// Voltage monitoring variables
float voltageLevel = 0.0;
unsigned long lastVoltageCheck = 0;
const float LOW_VOLTAGE_THRESHOLD = 6.5;  // Below this is considered low
const float HIGH_VOLTAGE_THRESHOLD = 7.5; // Above this is considered high

void setup() {
  Serial.begin(9600);

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  pinMode(voltagePin, INPUT);  // Initialize voltage monitoring pin
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  scanServo.attach(servoPin);
  scanServo.write(50);
  delay(300);
  
  // Initial voltage check
  checkVoltage();
}

void loop() {
  // Check voltage periodically (every 2 seconds)
  if (millis() - lastVoltageCheck >= 2000) {
    checkVoltage();
    adjustParametersBasedOnVoltage();
    lastVoltageCheck = millis();
  }

  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  // Servo scanning 
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

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop();
    delay(300);
  }

  if (approaching) {
    while (true) {
      distance = getDistance();
      if (distance <= scanThreshold || distance <= 0) break;

      leftIR = digitalRead(L_S);
      rightIR = digitalRead(R_S);

      if (leftIR == 1 || rightIR == 1) {
        Serial.println("Lost line during approach! Aborting.");
        Stop();
        delay(300);
        ultrasonicEnabled = true;
        approaching = false;
        return;
      }

      // Use voltage-adjusted speed for approach
      int slowApproachSpeed = map(forwardSpeed, 0, 255, 0, 180);
      analogWrite(enA, slowApproachSpeed);
      analogWrite(enB, slowApproachSpeed);
      forward();
      delay(100);
      Stop();
      delay(100);
    }

    Stop();
    delay(300);
    readColor();
    scanWithServo();
    handleDetectedColor(detectedColor);
    delay(500);
    ultrasonicEnabled = true;
    approaching = false;
    return;
  }

  // Regular line-following with voltage-adjusted speeds
  leftIR = digitalRead(L_S);
  rightIR = digitalRead(R_S);

  Serial.print("L_IR=");
  Serial.print(leftIR);
  Serial.print(" | R_IR=");
  Serial.print(rightIR);
  Serial.print(" | Distance=");
  Serial.print(getDistance());
  Serial.print(" | Voltage=");
  Serial.println(voltageLevel);

  if ((leftIR == 0) && (rightIR == 0)) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
  } else if ((leftIR == 0) && (rightIR == 1)) {
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
    turnRight();
    delay(50);
    delay(50);
  } else if ((leftIR == 1) && (rightIR == 0)) {
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
    delay(50);
    turnLeft();
    delay(50);
  } else {
    Stop();
  }

  delay(50); // stability delay
}

// New function to check battery voltage
void checkVoltage() {
  int sensorValue = analogRead(voltagePin);
  voltageLevel = sensorValue * (5.0 / 1023.0) * 2; // Assuming voltage divider with equal resistors
  
  Serial.print("Battery Voltage: ");
  Serial.print(voltageLevel);
  Serial.println("V");
}

// New function to adjust parameters based on voltage
void adjustParametersBasedOnVoltage() {
  if (voltageLevel < LOW_VOLTAGE_THRESHOLD) {
    // Low voltage - reduce speed to conserve power
    forwardSpeed = baseForwardSpeed * 0.7;
    turnSpeed = baseTurnSpeed * 0.7;
    Serial.println("Low voltage detected - reducing speed");
  } 
  else if (voltageLevel > HIGH_VOLTAGE_THRESHOLD) {
    // High voltage - can increase performance
    forwardSpeed = min(baseForwardSpeed * 1.3, 255); // Don't exceed 255
    turnSpeed = min(baseTurnSpeed * 1.3, 255);
    Serial.println("High voltage detected - increasing speed");
  }
  else {
    // Normal voltage - use base parameters
    forwardSpeed = baseForwardSpeed;
    turnSpeed = baseTurnSpeed;
  }
  
  // Additional voltage-dependent adjustments
  if (voltageLevel < 6.0) {
    // Critical low voltage - take emergency action
    Serial.println("CRITICAL VOLTAGE - SHUTTING DOWN");
    Stop();
    while(true); // Infinite loop to stop operation
  }
  
  Serial.print("Adjusted speeds - Forward: ");
  Serial.print(forwardSpeed);
  Serial.print(", Turn: ");
  Serial.println(turnSpeed);
}

// [Rest of your existing functions remain unchanged...]
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  int distance = duration * 0.034 / 2;
  return distance;
}

void readColor() {
  unsigned int red, green, blue;

  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blue = pulseIn(colorOut, LOW);

  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else if (blue < red && blue < green) detectedColor = "BLUE";
  else detectedColor = "UNKNOWN";

  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

void scanWithServo() {
  scanServo.write(50);
  delay(300);
  scanServo.write(100);
  Serial.println("Scanning left...");
  delay(500);
  scanServo.write(50);
  delay(300);
  scanServo.write(0);
  Serial.println("Scanning right...");
  delay(500);
  scanServo.write(50);
  Serial.println("Returning to center...");
  delay(500);
}
void driveForward(int cm, int speed = forwardSpeed) {
  int t = cm * 30;
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  forward();
  delay(t);
  Stop();
  delay(200);
}

void driveBackwards(int cm) {
  int estimatedTime = cm * 30;
  analogWrite(enA, forwardSpeed);
  analogWrite(enB, forwardSpeed);
  backward();
  delay(estimatedTime);
  Stop();
  delay(200);
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void driveCurveLeft(int cm) {
  int t = cm * 30;
  analogWrite(enA, forwardSpeed / 2);  // Left motor slower
  analogWrite(enB, forwardSpeed);      // Right motor normal
  forward();
  delay(t);
  Stop();
  delay(200);
}

void rotate180() {
  analogWrite(enA, forwardSpeed);
  analogWrite(enB, forwardSpeed);
  turnLeft();  // Rotate in place
  delay(800); // wait till rotate 180° is Done
  Stop();
  delay(200);
}


void handleDetectedColor(String color) {
  if (color == "RED") {
    turnRight();
    delay(1000);
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
    delay(400); // 20 cm
    Stop();
    delay(300);

    turnLeft();
    delay(500); // recentering
    Stop();
    delay(300);

    forward();
    delay(1000); // move 50 cm forward
    Stop();
    delay(300);

    turnLeft();
    delay(500); // scan for black line again
    Stop();
    delay(300);
  }

  else if (color == "GREEN") { //working fine now (tested_Abdo)
    //driveBackwards(20);
    driveForward(40, 100);   
    Stop();
    delay(300);
    driveBackwards(60);
  }

else if (color == "BLUE") {
  // Step 1: Reverse 30 cm
  driveBackwards(30);
  driveCurveLeft(30);  
  rotate180();
  Serial.println("BLUE maneuver complete → Parked");
  while (true);  // Stay parked
}
  else {
    Serial.println("Obstacle is " + color + " → Stopping");
    Stop();
  }
}