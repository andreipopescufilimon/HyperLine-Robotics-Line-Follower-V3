/* 
  > LF No Downforce
  > EDF/impeller off

  Last edited: 13th February 2025
  Last edited by: Popescu Filimon Andrei Cosmin
*/

#include "LF_SData.h"
#include "xmotionV3.h"

LF_SData lf_sdata;

#define DISTANCE_PIN A1
#define S0_PIN 0
#define S1_PIN 1
#define S2_PIN 2
#define S3_PIN 4
#define SIG_PIN A5

#define Start A0
#define DipSwitch1 5  // Dipswitch 1 for calibration mode
#define DipSwitch2 6  // Dipswitch 2 for debug mode
#define DipSwitch3 7  // Dipswitch 3 for safe mode

// PID constants
#define KP 0.22
#define KI 0.008
#define KD 9.2

float previousError = 0;
float integral = 0;

int setBaseSpeed = 0, baseSpeed = 0;
int setMaxSpeed = 0, maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
unsigned long startTime = 0, currentTime = 0, elapsedTime = 0, accelerationTime = 500;

void setup() {
  xmotion.UserLed1(100);

  // Setup pin modes for dip switches and start button
  pinMode(DipSwitch1, INPUT_PULLUP);
  pinMode(DipSwitch2, INPUT_PULLUP);
  pinMode(DipSwitch3, INPUT_PULLUP);
  pinMode(Start, INPUT_PULLUP);

  // Setup sensors and motors
  lf_sdata.setupDistanceSensor(DISTANCE_PIN);
  lf_sdata.setupLineSensors(S0_PIN, S1_PIN, S2_PIN, S3_PIN, SIG_PIN);


  // Determine mode based on dip switch settings
  if (digitalRead(DipSwitch3) == HIGH) {
    // Safe Run mode
    setBaseSpeed = 70;
    setMaxSpeed = 90;
  } else {
    setBaseSpeed = 90;
    setMaxSpeed = 100;
  }

  if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == LOW && digitalRead(DipSwitch3) == LOW) {
    // Calibration mode
    unsigned long startTime = millis();
    unsigned long calibrationTime = 5000;  // 5 seconds

    Serial.begin(9600);
    Serial.println("Starting calibration...");

    while (millis() - startTime < calibrationTime) {
      lf_sdata.calibrateSensors(true);
    }
    Serial.println("Calibration complete.");
  } else {
    lf_sdata.calibrateSensors(false);
  }

  xmotion.UserLed2(100);
  xmotion.SETUP();
  xmotion.ToggleLeds(100);

  if (digitalRead(DipSwitch2) == HIGH && digitalRead(DipSwitch1) == LOW && digitalRead(DipSwitch3) == LOW) {
    // Debug mode
    Serial.begin(9600);
  }
}

void loop() {
  if (startTime == 0) {
    startTime = millis();
  }

  elapsedTime = millis() - startTime;
  if (elapsedTime <= accelerationTime) {
    baseSpeed = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
    maxSpeed = map(elapsedTime, 0, accelerationTime, 0, setMaxSpeed);
  }

  long linePosition = lf_sdata.getLinePosition();
  float error = linePosition - 7500;

  // Update integral with last six errors for smoother calculation
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  //integral = error6 + error5 + error4 + error3 + error2 + error1 + error;
  integral = integral + error;

  float derivative = error - previousError;
  previousError = error;

  float correction = (KP * error) + (KI * integral) + (KD * derivative);

  int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

  if (digitalRead(DipSwitch2) == HIGH && digitalRead(DipSwitch1) == LOW && digitalRead(DipSwitch3) == LOW) {
    // Debug mode: print sensor data
    Serial.print("Line Position: ");
    Serial.println(linePosition);
  }

  if ((linePosition > 2000 && linePosition <= 4000) || (linePosition >= 11000 && linePosition < 13000)) {  // Medium turn
    baseSpeed = constrain(setBaseSpeed * 0.9, 50, setBaseSpeed);
    maxSpeed = constrain(setMaxSpeed * 0.9, 60, setMaxSpeed);
  } else {                     // Straight line
    baseSpeed = setBaseSpeed;  // Restore full speed
    maxSpeed = setMaxSpeed;
  }

  if (linePosition <= 2500) {
    xmotion.MotorControl(80, -70);
  } else if (linePosition >= 12500) {
    xmotion.MotorControl(-70, 80);
  } else {
    xmotion.MotorControl(map(leftMotorSpeed, 0, 100, 0, 255), map(rightMotorSpeed, 0, 100, 0, 255));
  }
}
