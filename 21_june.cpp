// === Motor control pins ===
const int mr1 = 2;  // Left motor IN1 (reversed wiring)
const int mr2 = 3;  // Left motor IN2 (reversed wiring)
const int ml1 = 4;  // Right motor IN1
const int ml2 = 5;  // Right motor IN2

// === Enable (PWM) pins ===
const int enA = 6;
const int enB = 9;

// === Ultrasonic sensor pins ===
const int trigPin = 8;
const int echoPin = 7;

float distance = 10;
bool avoidingObstacle = false;

unsigned long reengageStartTime = 0;
bool reengageInProgress = false;

// === IR sensor pins ===
const int leftIR  = A0;
const int rightIR = A1;

// === Color sensor pins ===
#define S0 A2
#define S1 A3
#define S2 A4
#define S3 A5
#define sensorOut 11
#define OE 13

// Color detection variables
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

int speedA = 70;
int speedB = 70;

// === Speed control ===
int add = 10;
int sub = 15;

void setup() {
  pinMode(mr1, OUTPUT); pinMode(mr2, OUTPUT);
  pinMode(ml1, OUTPUT); pinMode(ml2, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);

  pinMode(leftIR, INPUT); pinMode(rightIR, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Color sensor setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OE, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Initialize Output Enable (active low)
  digitalWrite(OE, LOW);
  
  // Set frequency scaling to 20% (S0=HIGH, S1=LOW)
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
    
    // Perform color detection when obstacle is detected
    String detectedColor = performColorDetection();
    Serial.print("Final Color Decision: ");
    Serial.println(detectedColor);
    
    // Handle obstacle based on color
    if (detectedColor == "RED") {
      Serial.println("RED detected - Performing manual overtaking");
      avoidingObstacle = true;
      handleObstacle();  // Your existing manual overtaking function
    }
    /*
    else if (detectedColor == "GREEN") {
      Serial.println("GREEN detected - Pushing through obstacle");
      pushThroughObstacle();
    }
    */
    else if (detectedColor == "BLUE") {
      Serial.println("BLUE detected - Pushing through obstacle");
      pushThroughObstacle();
    }
    else {
      Serial.println("Unknown color - Default overtaking");
      avoidingObstacle = true;
      handleObstacle();  // Default behavior
    }
    return;
  }

  // Normal line following logic
  int left = digitalRead(leftIR);   // HIGH = black line
  int right = digitalRead(rightIR);

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

// === Color Detection Functions ===

String performColorDetection() {
  Serial.println("Starting color detection - taking 10 readings...");
  
  // Arrays to store color counts
  int redCount = 0;
  int greenCount = 0;
  int blueCount = 0;
  int unknownCount = 0;
  
  // Take 10 readings with delay between each
  for (int i = 0; i < 10; i++) {
    String color = readSingleColor();
    
    Serial.print("Reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(color);
    
    // Count each color detection
    if (color == "RED") {
      redCount++;
    }
    else if (color == "GREEN") {
      greenCount++;
    }
    else if (color == "BLUE") {
      blueCount++;
    }
    else {
      unknownCount++;
    }
    
    delay(200);  // Wait between readings
  }
  
  // Print results
  Serial.print("Results - Red: ");
  Serial.print(redCount);
  Serial.print(", Green: ");
  Serial.print(greenCount);
  Serial.print(", Blue: ");
  Serial.print(blueCount);
  Serial.print(", Unknown: ");
  Serial.println(unknownCount);
  
  // Determine final color based on majority vote
  if (redCount >= greenCount && redCount >= blueCount && redCount >= 3) {
    return "RED";
  }
  else if (greenCount >= redCount && greenCount >= blueCount && greenCount >= 3) {
    return "GREEN";
  }
  else if (blueCount >= redCount && blueCount >= greenCount && blueCount >= 3) {
    return "BLUE";
  }
  else {
    return "UNKNOWN";
  }
}

String readSingleColor() {
  // Read Red frequency
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  
  delay(50);
  
  // Read Green frequency
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  
  delay(50);
  
  // Read Blue frequency
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  
  delay(50);
  
  return detectColor();
}

String detectColor() {
  // Check if object is actually present (minimum signal strength)
  int totalSignal = redFrequency + greenFrequency + blueFrequency;
  
  // If total signal is too high, no object is present (only ambient light)
  if (totalSignal > 3000) {
    return "NO OBJECT";
  }
  
  // If total signal is too low, sensor might be covered or malfunctioning
  if (totalSignal < 100) {
    return "SENSOR ERROR";
  }
  
  // Calculate the difference between colors for better detection
  int redGreenDiff = abs(redFrequency - greenFrequency);
  int redBlueDiff = abs(redFrequency - blueFrequency);
  int greenBlueDiff = abs(greenFrequency - blueFrequency);
  
  // Different thresholds for different colors
  int minDifference = 15;
  int greenMinDiff = 10;
  
  // Check if all colors are very similar first (white/gray object)
  if (redGreenDiff < 8 && redBlueDiff < 8 && greenBlueDiff < 8) {
    return "WHITE/GRAY";
  }
  
  // Find the minimum frequency (strongest color signal)
  int minFreq = min(redFrequency, min(greenFrequency, blueFrequency));
  
  // Enhanced GREEN detection logic
  if (greenFrequency == minFreq) {
    if ((redGreenDiff > greenMinDiff || greenBlueDiff > greenMinDiff)) {
      if (greenFrequency < (redFrequency * 0.9) || greenFrequency < (blueFrequency * 0.9)) {
        return "GREEN";
      }
    }
    if (greenFrequency < redFrequency && greenFrequency < blueFrequency) {
      float greenAdvantage = ((float)(redFrequency + blueFrequency) / 2) / greenFrequency;
      if (greenAdvantage > 1.05) {
        return "GREEN";
      }
    }
  }
  
  // RED detection
  if (redFrequency == minFreq && redFrequency < greenFrequency && redFrequency < blueFrequency) {
    if (redGreenDiff > minDifference && redBlueDiff > minDifference) {
      if (redFrequency < (greenFrequency * 0.8) && redFrequency < (blueFrequency * 0.8)) {
        return "RED";
      }
    }
  }
  
  // BLUE detection
  if (blueFrequency == minFreq && blueFrequency < redFrequency && blueFrequency < greenFrequency) {
    if (redBlueDiff > minDifference && greenBlueDiff > minDifference) {
      if (blueFrequency < (redFrequency * 0.8) && blueFrequency < (greenFrequency * 0.8)) {
        return "BLUE";
      }
    }
  }
  
  // Only use ratio detection if there's a significant color difference
  int maxDiff = max(redGreenDiff, max(redBlueDiff, greenBlueDiff));
  if (maxDiff > 20) {
    float redRatio = (float)redFrequency / totalSignal;
    float greenRatio = (float)greenFrequency / totalSignal;
    float blueRatio = (float)blueFrequency / totalSignal;
    
    if (greenRatio < redRatio && greenRatio < blueRatio) {
      if ((redRatio - greenRatio) > 0.05 && (blueRatio - greenRatio) > 0.05) {
        return "GREEN";
      }
    }
    else if (redRatio < greenRatio && redRatio < blueRatio) {
      if ((greenRatio - redRatio) > 0.05 && (blueRatio - redRatio) > 0.05) {
        return "RED";
      }
    }
    else if (blueRatio < redRatio && blueRatio < greenRatio) {
      if ((redRatio - blueRatio) > 0.05 && (greenRatio - blueRatio) > 0.05) {
        return "BLUE";
      }
    }
  }
  
  return "UNKNOWN";
}

// === New Obstacle Handling Functions ===

void pushThroughObstacle() {
  Serial.println("Pushing blue obstacle forward...");
  
  // Move forward with full power to push obstacle away
  moveForward(100, 100);
  delay(600);  // Push forward for 2 seconds
  
  stopMotors();
  delay(300);
  
  Serial.println("Returning to original position...");
  
  // Move backward to return to original position
  moveBackward(100, 100);
  delay(600);  // Move back for 2 seconds
  
  stopMotors();
  delay(300);
  
  Serial.println("Obstacle cleared! Resuming line following");
}

void stopForever() {
  Serial.println("STOP SIGNAL DETECTED - ROBOT STOPPED PERMANENTLY");
  while (true) {
    stopMotors();
    delay(1000);
    // Robot will stay here forever
  }
}

// === Original Movement Functions ===

void moveForward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);

  digitalWrite(mr1, LOW);
  digitalWrite(mr2, HIGH);  // Left motor forward (reversed wiring)

  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);   // Right motor forward
}

void moveBackward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);

  digitalWrite(mr1, HIGH);
  digitalWrite(mr2, LOW);  // Left motor backward (reversed wiring)

  digitalWrite(ml1, LOW);
  digitalWrite(ml2, HIGH);  // Right motor backward
}

void leftForward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, HIGH);  // Left forward (reversed wiring)
}

void leftBackward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(mr1, HIGH);
  digitalWrite(mr2, LOW);  // Left backward (reversed wiring)
}

void rightForward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);  // Right forward
}

void rightBackward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(ml1, LOW);
  digitalWrite(ml2, HIGH);  // Right backward
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(mr1, LOW); digitalWrite(mr2, LOW);
  digitalWrite(ml1, LOW); digitalWrite(ml2, LOW);
}

long readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);  // Timeout after 20 ms (max ~3.4m)

  if (duration == 0) return -1;  // No echo received
  long distance = duration * 0.034 / 2;

  if (distance > 300 || distance <= 0) return -1;  // Invalid range (>3m or <=0)
  
  return distance;  // Valid distance in cm
}

void handleObstacle() {
  stopMotors(); delay(300);

  moveBackward(80, 80); delay(500); stopMotors(); delay(300);
  moveWithTurn(-180, 180); delay(180); stopMotors(); delay(300);
  moveForward(65, 65); delay(1600); stopMotors(); delay(300);
  moveWithTurn(200, -200); delay(280); stopMotors(); delay(300);
  moveForward1(80); delay(300); 

  waitForLineDetection();
  avoidingObstacle = false;
}

void moveWithTurn(int leftSpeed, int rightSpeed) {
  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));

  digitalWrite(mr1, leftSpeed >= 0 ? LOW : HIGH);
  digitalWrite(mr2, leftSpeed >= 0 ? HIGH : LOW);

  digitalWrite(ml1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(ml2, rightSpeed >= 0 ? LOW : HIGH);
}

void waitForLineDetection() {
  Serial.println("Searching for line...");
  while (true) {
    int l = digitalRead(leftIR);
    int r = digitalRead(rightIR);

    if (l == 1 || r == 1) {
      Serial.println("Line detected!");
      break;
    } else {
      moveForward1(50);
    }
    delay(10);
  }
}

void moveForward1(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);

  // Left motor forward
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, HIGH);

  // Right motor forward
  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);
}