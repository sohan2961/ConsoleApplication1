#include <Servo.h> 
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A4
#define R_S A5
#define trigPin 12
#define echoPin 13
#define S0 A2
#define S1 A1
#define S2 0
#define S3 1
#define colorOut A0
#define servoPin 4
#define SERVO_CENTER 70

Servo scanServo;

int forwardSpeed = 70;
int turnSpeed = 80;
int curveDelay = 150;
int approachThreshold = 20;
int scanThreshold = 3;
int approachSpeed = 80;
bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

void forward();
void backward();
void turnLeft();
void turnRight();
void Stop();
void curveLeft();
void curveRight();
void approachToObstacle();
void readColor();
void handleDetectedColor(String color);
void driveForward(int cm, int speed = 70);
void driveBackwards(int cm, int speed = 70);
void rotate90();
void rotateRight90();
void approachRightToLane();
int getDistance();

void setup() {
  Serial.begin(9600);
  pinMode(R_S, INPUT); pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);
  scanServo.attach(servoPin);
  scanServo.write(SERVO_CENTER);
  delay(300);
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = getDistance();

  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(distance);

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop(); delay(300);
  }

  if (approaching) {
    approachToObstacle();
    return;
  }

  if (distance > 0 && distance <= approachThreshold) {
    Stop();
    Serial.println("Obstacle ahead! Stopping.");
    delay(500);
    return;
  }

  if (leftIR == 0 && rightIR == 0) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
  } else if (leftIR == 1 && rightIR == 0) {
    Stop(); delay(30);
    curveLeft();
  } else if (leftIR == 0 && rightIR == 1) {
    curveRight();
    Stop(); delay(30);
  } else {
    Stop();
    delay(100);
  }
}

void approachToObstacle() {
  int l, r, d;
  unsigned long stepTime = 30;
  while (true) {
    d = getDistance();
    l = digitalRead(L_S); r = digitalRead(R_S);
    if (l == 1 && r == 1) {
      Serial.println("Lost line during approach! Aborting.");
      Stop(); delay(300); ultrasonicEnabled = true; approaching = false; return;
    }
    if (d <= scanThreshold || d <= 0) break;
    if (l == 0 && r == 0) {
      analogWrite(enA, approachSpeed); analogWrite(enB, approachSpeed);
      forward(); delay(stepTime); Stop();
    } else if (l == 1 && r == 0) {
      analogWrite(enA, 0); analogWrite(enB, approachSpeed);
      digitalWrite(in1, LOW); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
      delay(stepTime); Stop();
    } else if (l == 0 && r == 1) {
      analogWrite(enA, approachSpeed); analogWrite(enB, 0);
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, LOW);
      delay(stepTime); Stop();
    }
    delay(10);
  }
  Stop(); delay(300); readColor(); handleDetectedColor(detectedColor);
  delay(500); approaching = false;
}

void readColor() {
  unsigned int red, green, blue;
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); blue = pulseIn(colorOut, LOW);
  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else detectedColor = "OTHER";
  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

void handleDetectedColor(String color) {
  int colorScenarioSpeed = 80;
  if (color == "RED") {
    driveBackwards(12, colorScenarioSpeed);
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    if (leftDist > 20) {
      ultrasonicEnabled = false;
      rotate90(); delay(50);
      driveForward(30, colorScenarioSpeed); delay(200);
      rotateRight90(); delay(200);
      driveForward(20, colorScenarioSpeed);
      approachRightToLane();
      ultrasonicEnabled = true;
      Serial.println("RED: Path left is clear, performed left-right bypass.");
    } else {
      Serial.println("RED: Left not clear, only backed up and stopped.");
    }
    scanServo.write(SERVO_CENTER);
    Stop();
    Serial.println("RED scenario complete.");
  } else if (color == "GREEN") {
    Serial.println("GREEN scenario: Forward 40cm, then reverse 40cm on lane, ignore ultrasonic for 5s.");
    driveForward(20, 150);
    Stop(); delay(300);
    driveBackwards(20, 70);
    Stop(); delay(300);
    ultrasonicEnabled = false;
    unsigned long ignoreStart = millis();
    Serial.println("Ultrasonic disabled for 5 seconds...");
    while (millis() - ignoreStart < 5000) {
      int l = digitalRead(L_S);
      int r = digitalRead(R_S);
      if (l == 0 && r == 0) {
        analogWrite(enA, forwardSpeed);
        analogWrite(enB, forwardSpeed);
        forward();
      } else if (l == 1 && r == 0) {
        Stop(); delay(30);
        curveLeft();
      } else if (l == 0 && r == 1) {
        curveRight();
        Stop(); delay(30);
      } else {
        Stop();
        delay(100);
      }
      delay(20);
    }
    ultrasonicEnabled = true;
    Serial.println("Ultrasonic re-enabled, GREEN scenario complete: Lane reacquired.");
    Stop();
  } else {
    Serial.println("OTHER COLOR scenario: Back 10cm, scan right/left, park if right is clear.");
    driveBackwards(10, colorScenarioSpeed);
    delay(200);
    scanServo.write(0); delay(400);
    int rightDist = getDistance();
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    Serial.print("Scan distances: Right="); Serial.print(rightDist); Serial.print(", Left="); Serial.println(leftDist);
    if (rightDist > 20) {
      rotateRight90(); delay(200);
      Stop();
      Serial.println("Parked to the right.");
      while (1) { Stop(); }
    } else {
      Serial.println("Right not clear, not parking.");
    }
  }
}

void approachRightToLane() {
  int l, r;
  int leftSpeed = 65;
  int rightSpeed = 40;
  while (true) {
    l = digitalRead(L_S);
    r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      Stop();
      delay(100);
      Serial.println("ApproachRightToLane: Black detected, resuming normal lane following.");
      break;
    }
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    delay(120); Stop(); delay(5);
  }
  Stop();
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void curveLeft() {
  analogWrite(enA, turnSpeed);
  
  turnLeft();
  delay(curveDelay);
  Stop();
}

void curveRight() {
  
  analogWrite(enB, turnSpeed);
  turnRight();
  delay(curveDelay);
  Stop();
}

void driveForward(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  forward();
  delay(t);
  Stop(); delay(200);
}

void driveBackwards(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  backward();
  delay(t);
  Stop(); delay(200);
}

void rotate90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(500);
  Stop(); delay(200);
}

void rotateRight90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(500);
  Stop(); delay(200);
}

int getDistance() {
  if (!ultrasonicEnabled) return 1000;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

###########################################################################################################
#include <Servo.h>
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A4
#define R_S A5
#define trigPin 12
#define echoPin 13
#define servoPin 4
#define S0 A2
#define S1 A1
#define S2 0
#define S3 1
#define sensorOut A0
#define OE 11

float distance = 10;
bool avoidingObstacle = false;
unsigned long reengageStartTime = 0;
bool reengageInProgress = false;
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
int speedA = 58;
int speedB = 58;
int add = 10;
int sub = 15;

void setup() {
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(L_S, INPUT); pinMode(R_S, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OE, OUTPUT); pinMode(sensorOut, INPUT);
  digitalWrite(OE, LOW);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
  Serial.println("Robot with Color Detection Started");
}

void loop() {
  distance = readUltrasonic();
  if (distance > 0 && distance <= 6) {
    stopMotors();
    delay(500);
    String detectedColor = performColorDetection();
    Serial.print("Final Color Decision: ");
    Serial.println(detectedColor);
    if (detectedColor == "RED") {
      Serial.println("RED detected - Performing manual overtaking");
      avoidingObstacle = true;
      handleObstacle();
    }
    else if (detectedColor == "BLUE") {
      Serial.println("BLUE detected - Pushing through obstacle");
      pushThroughObstacle();
    }
    else {
      Serial.println("Unknown color - Default overtaking");
      avoidingObstacle = true;
      handleObstacle();
    }
    return;
  }
  int left = digitalRead(L_S);
  int right = digitalRead(R_S);
  Serial.print("Left IR: "); Serial.print(left);
  Serial.print(" | Right IR: "); Serial.println(right);
  if (left == 0 && right == 0) {
    moveForward(speedA, speedB);
  }
  else if (left == 0 && right == 1) {
    leftForward(80);
    rightBackward(60);
  }
  else if (left == 1 && right == 0) {
    leftBackward(60);
    rightForward(80);
  }
  else {
    stopMotors();
  }
}

String performColorDetection() {
  Serial.println("Starting color detection - taking 10 readings...");
  int redCount = 0, greenCount = 0, blueCount = 0, unknownCount = 0;
  for (int i = 0; i < 10; i++) {
    String color = readSingleColor();
    Serial.print("Reading "); Serial.print(i + 1); Serial.print(": "); Serial.println(color);
    if (color == "RED") redCount++;
    else if (color == "GREEN") greenCount++;
    else if (color == "BLUE") blueCount++;
    else unknownCount++;
    delay(200);
  }
  Serial.print("Results - Red: "); Serial.print(redCount);
  Serial.print(", Green: "); Serial.print(greenCount);
  Serial.print(", Blue: "); Serial.print(blueCount);
  Serial.print(", Unknown: "); Serial.println(unknownCount);
  if (redCount >= greenCount && redCount >= blueCount && redCount >= 3) return "RED";
  else if (greenCount >= redCount && greenCount >= blueCount && greenCount >= 3) return "GREEN";
  else if (blueCount >= redCount && blueCount >= greenCount && blueCount >= 3) return "BLUE";
  else return "UNKNOWN";
}

String readSingleColor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  return detectColor();
}

String detectColor() {
  int totalSignal = redFrequency + greenFrequency + blueFrequency;
  if (totalSignal > 3000) return "NO OBJECT";
  if (totalSignal < 100) return "SENSOR ERROR";
  int redGreenDiff = abs(redFrequency - greenFrequency);
  int redBlueDiff = abs(redFrequency - blueFrequency);
  int greenBlueDiff = abs(greenFrequency - blueFrequency);
  int minDifference = 15, greenMinDiff = 10;
  if (redGreenDiff < 8 && redBlueDiff < 8 && greenBlueDiff < 8) return "WHITE/GRAY";
  int minFreq = min(redFrequency, min(greenFrequency, blueFrequency));
  if (greenFrequency == minFreq) {
    if ((redGreenDiff > greenMinDiff || greenBlueDiff > greenMinDiff)) {
      if (greenFrequency < (redFrequency * 0.9) || greenFrequency < (blueFrequency * 0.9)) return "GREEN";
    }
    if (greenFrequency < redFrequency && greenFrequency < blueFrequency) {
      float greenAdvantage = ((float)(redFrequency + blueFrequency) / 2) / greenFrequency;
      if (greenAdvantage > 1.05) return "GREEN";
    }
  }
  if (redFrequency == minFreq && redFrequency < greenFrequency && redFrequency < blueFrequency) {
    if (redGreenDiff > minDifference && redBlueDiff > minDifference) {
      if (redFrequency < (greenFrequency * 0.8) && redFrequency < (blueFrequency * 0.8)) return "RED";
    }
  }
  if (blueFrequency == minFreq && blueFrequency < redFrequency && blueFrequency < greenFrequency) {
    if (redBlueDiff > minDifference && greenBlueDiff > minDifference) {
      if (blueFrequency < (redFrequency * 0.8) && blueFrequency < (greenFrequency * 0.8)) return "BLUE";
    }
  }
  int maxDiff = max(redGreenDiff, max(redBlueDiff, greenBlueDiff));
  if (maxDiff > 20) {
    float redRatio = (float)redFrequency / totalSignal;
    float greenRatio = (float)greenFrequency / totalSignal;
    float blueRatio = (float)blueFrequency / totalSignal;
    if (greenRatio < redRatio && greenRatio < blueRatio) {
      if ((redRatio - greenRatio) > 0.05 && (blueRatio - greenRatio) > 0.05) return "GREEN";
    }
    else if (redRatio < greenRatio && redRatio < blueRatio) {
      if ((greenRatio - redRatio) > 0.05 && (blueRatio - redRatio) > 0.05) return "RED";
    }
    else if (blueRatio < redRatio && blueRatio < greenRatio) {
      if ((redRatio - blueRatio) > 0.05 && (greenRatio - blueRatio) > 0.05) return "BLUE";
    }
  }
  return "UNKNOWN";
}

void pushThroughObstacle() {
  Serial.println("Pushing blue obstacle forward...");
  moveForward(100, 100);
  delay(600);
  stopMotors();
  delay(300);
  Serial.println("Returning to original position...");
  moveBackward(100, 100);
  delay(600);
  stopMotors();
  delay(300);
  Serial.println("Obstacle cleared! Resuming line following");
}

void stopForever() {
  Serial.println("STOP SIGNAL DETECTED - ROBOT STOPPED PERMANENTLY");
  while (true) {
    stopMotors();
    delay(1000);
  }
}

// *** FIXED DIRECTION MOTOR CONTROL FUNCTIONS (SWAP LOGIC) ***
void moveForward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);
  // SWAP LOGIC: Reverse the HIGH/LOW for both motors to match your wiring
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveBackward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void leftForward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void leftBackward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void rightForward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void rightBackward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

long readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  if (distance > 300 || distance <= 0) return -1;
  return distance;
}

void handleObstacle() {
  stopMotors(); delay(300);
  moveBackward(80, 80); delay(500); stopMotors(); delay(300);
  leftBackward(180); rightForward(180); delay(180); stopMotors(); delay(300);
  moveForward(65, 65); delay(1600); stopMotors(); delay(300);
  leftForward(200); rightBackward(200); delay(280); stopMotors(); delay(300);
  moveForward1(80); delay(300);
  waitForLineDetection();
  avoidingObstacle = false;
}

void moveForward1(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void waitForLineDetection() {
  Serial.println("Searching for line...");
  while (true) {
    int l = digitalRead(L_S);
    int r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      Serial.println("Line detected!");
      break;
    } else {
      moveForward1(50);
    }
    delay(10);
  }
}