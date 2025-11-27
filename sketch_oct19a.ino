#define m1p1 5
#define m1p2 6

#define m2p1 9
#define m2p2 10

unsigned long startTime;
unsigned long ignoreBlackDuration = 8700;

float Kp = 0.6;
//float Kp = 6;
float  Kd= 0.005;
float dt = 0.01; // 10 ms
#define rightMaxSpeed 255
#define leftMaxSpeed 255

int inclination = 0 ;
int leftBaseSpeed = 96;
int rightBaseSpeed = 107;

#define LED_PIN 13   // Use the built-in LED for signaling
#define BUFFER_SIZE 10
int rightSensorValues[BUFFER_SIZE];
int leftSensorValues[BUFFER_SIZE];


#define NUM_SENSORS 6
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
long whiteValues[NUM_SENSORS];  // readings on white
long blackValues[NUM_SENSORS];  // readings on black
int threshold[NUM_SENSORS];     // computed thresholds
#define LED_PIN 13
//int weights[NUM_SENSORS] = {15, 5, 2, -2, -5, -15};
//int weights[NUM_SENSORS] = {680, 390, 50, -50, -390, -680}; plus que parfait


int weights[NUM_SENSORS] = {680, 385, 50, -50, -385, -680};
int weightsRightBias[NUM_SENSORS] = {-800, -500, -100, 20, 40, 80};
int *activeWeights = weights; // left to right
int lastError = 0;



void calibrate() {
  Serial.println("=== CALIBRATION START ===");
 
  for (int i = 0; i < NUM_SENSORS; i++) {
    whiteValues[i] = 0;
    blackValues[i] = 0;
}

  // --- White surface ---
  Serial.println("Place sensors on WHITE surface...");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      whiteValues[i] += analogRead(sensorPins[i]);
    }
    // Blink LED during reading
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }

  // Average white values
  for (int i = 0; i < NUM_SENSORS; i++) whiteValues[i] /= 50;

  Serial.println("White calibration done! Move robot to BLACK line now.");
 
  // LED ON for 2 seconds → signal to switch to black
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);

  // --- Black surface ---
  Serial.println("Place sensors on BLACK line...");
  for (int i = 0; i < NUM_SENSORS; i++) blackValues[i] = 0;

  startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      blackValues[i] += analogRead(sensorPins[i]);
    }
    // Blink LED during reading
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }

  // Average black values
  for (int i = 0; i < NUM_SENSORS; i++) blackValues[i] /= 50;

  Serial.println("Black calibration done!");
  for (int i = 0; i < NUM_SENSORS; i++) {
  threshold[i] = (whiteValues[i] + blackValues[i]) / 2;
}
 
  // LED ON → calibration complete
  digitalWrite(LED_PIN, HIGH);
  Serial.println("=== CALIBRATION COMPLETE ===");
}
void setup() {
  // --- Motor pins ---
  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);
 

  // --- LED + calibration button (optional) ---
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(3, INPUT_PULLUP); // Button on pin 3, optional if you have one

  // --- Serial communication ---
  Serial.begin(9600);
  Serial.println("Starting setup...");

  // --- Run sensor calibration ---
  calibrate();

  // --- Initialize motor-related variables (if any) ---
  for (int i = 0; i < BUFFER_SIZE; i++) {
    rightSensorValues[i] = 0;
    leftSensorValues[i] = 0;
  }

  // --- Small delay before starting movement ---
  Serial.println("Calibration done. Starting in 2 seconds...");
  delay(2000);

  // --- Move forward slowly to start ---
  analogWrite(m1p1, 120); // Right motor forward
  analogWrite(m1p2, 0);
  analogWrite(m2p1, 100); // Left motor forward
  analogWrite(m2p2, 0);

  delay(500);

  // Stop motors for a moment before main loop begins
  analogWrite(m1p1, 0);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, 0);
  analogWrite(m2p2, 0);

  Serial.println("Setup complete!");
  startTime = millis();
}


float calculateLinePosition() {
    long weightedSum = 0;
    int totalOnLine = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = analogRead(sensorPins[i]);
        int sensorOnLine = (value < threshold[i]) ? 1 : 0; // 1 = black
        weightedSum += sensorOnLine * activeWeights[i];
        totalOnLine += sensorOnLine;
    }

    if (totalOnLine == 0) return lastError; // keep last error if lost
    return (float)weightedSum / totalOnLine;
}

void PID() {
  float position = calculateLinePosition();
  float error = position - inclination;
  float diff = (error - lastError) / dt;

  float motorAdjust = Kp * error + Kd * diff;

  int rightSpeed = rightBaseSpeed;
  int leftSpeed  = leftBaseSpeed;

  // --- If motorAdjust exceeds base speed → use reverse on one wheel ---
  /*if (motorAdjust > rightBaseSpeed) {
    // Turn left sharply: left wheel goes backward
    leftSpeed  = -min((int)motorAdjust, leftMaxSpeed); // backward
    rightSpeed = rightBaseSpeed;                        // forward
  } else if (motorAdjust < -leftBaseSpeed) {
    // Turn right sharply: right wheel goes backward
    rightSpeed = -min((int)(-motorAdjust), rightMaxSpeed); // backward
    leftSpeed  = leftBaseSpeed;                             // forward
  } */
 
    // Normal PD control
    rightSpeed = rightBaseSpeed - motorAdjust;
    leftSpeed  = leftBaseSpeed + motorAdjust;
 

  // --- Constrain speeds ---
  rightSpeed = constrain(rightSpeed, -rightMaxSpeed, rightMaxSpeed);
  leftSpeed  = constrain(leftSpeed, -leftMaxSpeed, leftMaxSpeed);

  // --- Write to motors ---
  if (rightSpeed >= 0) {
    analogWrite(m1p1, rightSpeed);
    analogWrite(m1p2, 0);
  } else {
    analogWrite(m1p1, 0);
    analogWrite(m1p2, -rightSpeed);
  }

  if (leftSpeed >= 0) {
    analogWrite(m2p1, leftSpeed);
    analogWrite(m2p2, 0);
  } else {
    analogWrite(m2p1, 0);
    analogWrite(m2p2, -leftSpeed);
  }

  lastError = error;
}


bool startedHex = false;
bool alreadyTurned = false;
int i = 0;

void loop() {
  if ((!alreadyTurned) && ((millis() - startTime) > ignoreBlackDuration) && (i < 4)) {
    if (allSensorsBlackStable()) {
      startedHex = true;
      // switch to right-biased weights
      activeWeights = weightsRightBias;

      // extend the ignore window for next period
      ignoreBlackDuration += 1000;
      i++;

      PID();
      delay(10);

      // after each small turn, revert to normal weights to stabilize
      activeWeights = weights;
      delay(10);

      // stop when 3 pulses are done
      // if (i == 4) {
      //   ignoreBlackDuration +=5000; }
      if (i==4) {alreadyTurned = true;
      }
      return;
    }
  }

  // Normal line following
  if(!startedHex)activeWeights = weights;
  else
    activeWeights = weightsRightBias;
  PID();
  delay(30);
}



a

bool allSensorsBlackStable() {
  int blackCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (analogRead(sensorPins[i]) < threshold[i]) blackCount++;
  }
  if (blackCount == NUM_SENSORS) blackFrames++;
  else blackFrames = 0;


  return (blackFrames >= 3); // must stay black for 3 frames
}