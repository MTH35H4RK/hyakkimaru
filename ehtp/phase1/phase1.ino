#include <QTRSensors.h>

volatile int pos;

////////////////////part algo//////////////////////////
#include "Dictionary.h"

const int maxDigits = 10;  // Adjust this according to the maximum number of digits you expect
int numbers[maxDigits];
int degrees[maxDigits];
int numbersLength = 0;

float bigForward = 0.42;
float smallForward = 0.295;
float rotate90 = 0.14 ;
float rotate180 = 0.134 * 2 ;
float rotate45 = 0.135 / 2 ;
float rotate135 = rotate45 + rotate90;

int travelTime[maxDigits * 2];
float velocities[maxDigits * 2][2];

Dictionary<int, float> m;

// Map direction numbers to degrees
int directionDegrees[] = {0, 0, 45, 90, 135, 180, -135, -90, -45};
int current_angle = 0;
//////////////////////////////////////////////////////

QTRSensors qtr;
QTRSensors qtr2;

int FlagLineFollow = 0;
const uint8_t SensorCount = 8;
const uint8_t SensorCount2 = 8;

uint16_t sensorValues[SensorCount];
uint16_t sensorValues2[SensorCount2];

#define ENCA 3
#define ENCB 11
const int enca[] = { 5, 6} ;
const int encb[] = { 3, 12};
const int pwm[] = { 10, 5};
const int in1[] = { 8, 7};
const int in2[] = { 9, 6};

const int BASE_SPEED = 50;
const int MAX_SPEED = 65;

float Kp = 0.1;
float Ki = 0;
float Kd = 0.14;
int lastError = 0;
int integral = 0;

uint16_t position;
uint16_t position1;
uint16_t position2;

void setMotors(int speedA, int speedB) {
  if (speedA < 5 && speedA >= 0) {
    digitalWrite(in2[0], HIGH);
    digitalWrite(in1[0], HIGH);
    analogWrite(pwm[0], 0);
  }
  if (speedB < 5 && speedB >= 0) {
    digitalWrite(in2[1], HIGH);
    digitalWrite(in1[1], HIGH);
    analogWrite(pwm[1], 0);
  }
  if (speedA < 0) {
    digitalWrite(in2[0], HIGH);
    digitalWrite(in1[0], LOW);
    analogWrite(pwm[0], -speedA);
  } else {
    digitalWrite(in2[0], LOW);
    digitalWrite(in1[0], HIGH);
    analogWrite(pwm[0], speedA);
  }

  if (speedB < 0) {
    digitalWrite(in2[1], HIGH);
    digitalWrite(in1[1], LOW);
    analogWrite(pwm[1], -speedB);
  } else {
    digitalWrite(in2[1], LOW);
    digitalWrite(in1[1], HIGH);
    analogWrite(pwm[1], speedB);
  }
}

void setup() {
  Serial.begin(9600);

  m[0] = 0;
  m[45] = rotate45;
  m[90] = rotate90;
  m[135] = rotate135;
  m[-45] = -rotate45;
  m[-90] = -rotate90;
  m[-135] = -rotate135;
  m[180] = rotate180;

  pinMode(in2[1], OUTPUT);
  pinMode(in1[1], OUTPUT);
  pinMode(in2[0], OUTPUT);
  pinMode(in1[0], OUTPUT);
  pinMode(pwm[0], OUTPUT);
  pinMode(pwm[1], OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(44, OUTPUT);
  digitalWrite(52, LOW);
  digitalWrite(50, LOW);
  digitalWrite(48, LOW);
  digitalWrite(46, LOW);
  digitalWrite(44, LOW);

  qtr.setTypeRC();
  qtr2.setTypeRC();

  qtr.setSensorPins((const uint8_t[]) {
    A15, A14, A13, A12, A11, A10, A9, A8
  }, SensorCount);
  qtr2.setSensorPins((const uint8_t[]) {
    A7, A6, A5, A4, A3, A2, A1, A0
  }, SensorCount2);

  qtr.setEmitterPin(23);
  qtr2.setEmitterPin(22);

  pinMode(49, OUTPUT);
  digitalWrite(49, HIGH);

  Serial.println("Start");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');  // Read the incoming data until newline character
      input.trim();  // Remove any leading or trailing whitespace

      if (input.startsWith("M5")) {
        input = input.substring(2);  // Remove "M5" from the input

        // Clear the previous numbers array
        memset(numbers, 0, sizeof(numbers));
        numbersLength = 0;

        for (int i = 0; i < input.length(); i++) {
          if (isDigit(input[i])) {
            if (numbersLength < maxDigits) {
              numbers[numbersLength++] = input[i] - '0';  // Convert char to int and store in array
            } else {
              Serial.println("Error: Input exceeds maximum digits length.");
              break;
            }
          }
        }

        // Calculate the degrees array
        calculateDegrees();
current_angle = degrees[0];
        

        // Print the numbers array
        Serial.print("Received numbers: ");
        for (int i = 0; i < numbersLength; i++) {
          Serial.print(numbers[i]);
          if (i < numbersLength - 1) {
            Serial.print(", ");
          }
        }
        Serial.println();

        break;  // Exit the while loop after successfully reading the data
      } else {
        Serial.println("Data does not start with 'M5'. Waiting for valid data...");
      }
    }
  }

  int j = 0;
  for (int i = 0; i < numbersLength; i++) {
    velocities[j][0] = m[degrees[i]];
    velocities[j][1] = -m[degrees[i]];
    travelTime[j] = 1;
    j++;
    if ((degrees[i] + current_angle) % 10 == 0) {
      velocities[j][0] = smallForward;
      velocities[j][1] = smallForward;
    } else {
      velocities[j][0] = bigForward;
      velocities[j][1] = bigForward;
    }
    current_angle += degrees[i];
    travelTime[j] = 2;
    j++;
  }

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
}

void pidJ() {
  qtr.read(sensorValues);
  qtr2.read(sensorValues2);

  if (sensorValues2[0] > 1500) {
    position = (1 * 1000) + 7000;
  } else if (sensorValues[7] > 1500) {
    position = 7 * 1000;
  } else if (sensorValues2[7] > 1500) {
    position = (7 * 1000) + 7000;
  } else if (sensorValues[0] > 1500) {
    position = 0 * 1000;
  } else if (sensorValues2[6] > 1500) {
    position = (6 * 1000) + 7000;
  } else if (sensorValues[1] > 1500) {
    position = 1 * 1000;
  } else if (sensorValues2[5] > 1500) {
    position = (5 * 1000) + 7000;
  } else if (sensorValues[2] > 1500) {
    position = 2 * 1000;
  } else if (sensorValues2[4] > 1500) {
    position = (4 * 1000) + 7000;
  } else if (sensorValues[3] > 1500) {
    position = 3 * 1000;
  } else if (sensorValues2[3] > 1500) {
    position = (3 * 1000) + 7000;
  } else if (sensorValues[4] > 1500) {
    position = 4 * 1000;
  } else if (sensorValues2[2] > 1500) {
    position = (2 * 1000) + 7000;
  } else if (sensorValues[5] > 1500) {
    position = 5 * 1000;
  } else if (sensorValues2[1] > 1500) {
    position = (1 * 1000) + 7000;
  } else if (sensorValues[6] > 1500) {
    position = 6 * 1000;
  }

  int error = position - 7000;
  integral += error;
  integral = constrain(integral, -10000, 10000);
  int derivative = error - lastError;
  int speedDifference = Kp * error + Ki * integral + Kd * derivative;

  int speedA = BASE_SPEED + speedDifference;  // Right
  int speedB = BASE_SPEED - speedDifference;  // Left

  speedA = constrain(speedA, -40, MAX_SPEED);
  speedB = constrain(speedB, -40, MAX_SPEED);

  setMotors(speedA, speedB);

  lastError = error;
}

int inter = 0;
int kindex = 1;

void loop() {
  readinter();
  Serial.print("#");
  Serial.println(inter);

  if (inter > 3) {
    Serial.println(degrees[kindex]);
    Serial.println("#########");
    if(kindex==numbersLength){
      setMotors(0, 0);
      delay(999999);
    }
    switch (degrees[kindex++]) {
      case 45:
        setMotors(30, 30);
        delay(250);
        setMotors(56, -44);
        delay(390/2);  // Adjust delay to match the time required to turn 45 degrees
        setMotors(0, 0);
        break;
      case 90:
        setMotors(30, 30);
        delay(200);
        setMotors(56, -44);
        delay(700/1.4);  // Adjust delay to match the time required to turn 90 degrees
        setMotors(0, 0);
        break;
      case 135:
        setMotors(30, 30);
        delay(100);
        setMotors(56, -46);
        delay(890);  // Adjust delay to match the time required to turn 135 degrees
        setMotors(0, 0);
        break;
      case 180:
        setMotors(30, 30);
        delay(300);
        setMotors(55, -55);
        delay(1000);  // Adjust delay to match the time required to turn 180 degrees
        setMotors(0, 0);
        break;
      case -45:
        setMotors(40, 40);
        delay(280);
        setMotors(-56, 44);
        delay(370);  // Adjust delay to match the time required to turn -45 degrees
        setMotors(0, 0);
        break;
      case -90:
        setMotors(30, 30);
        delay(290);
        setMotors(-56, 44);
        delay(600);  // Adjust delay to match the time required to turn -90 degrees
        setMotors(0, 0);
        break;
      case -135:
        setMotors(35, 35);
        delay(250);
        setMotors(-56, 44);
        delay(950/1.2);  // Adjust delay to match the time required to turn -135 degrees
        setMotors(0, 0);
        break;
      case 0:
        setMotors(40, 40);
        delay(750);
        while (inter > 3) {
          readinter();
          setMotors(50, 50);
        }
        break;
    }
  } else {
    pidJ();
  }
}

void calculateDegrees() {
  int currentOrientation = 0;  // Assuming initial orientation is 0 degrees

  for (int i = 0; i < numbersLength; i++) {
    int targetOrientation = directionDegrees[numbers[i]];
    int turn = targetOrientation - currentOrientation;

    // Adjust turn to be within the range [-180, 180]
    if (turn > 180) {
      turn -= 360;
    } else if (turn < -180) {
      turn += 360;
    }

    degrees[i] = turn;
    currentOrientation = targetOrientation;
  }
}

void readinter() {
  qtr.read(sensorValues);
  qtr2.read(sensorValues2);
  inter = 0;
  // Process sensor values for the first array
  if (sensorValues2[0] > 1500) {
    inter++;
  } if (sensorValues[7] > 1500) {
    inter++;
  }  if (sensorValues2[7] > 1500) {
    inter++;
  }  if (sensorValues[0] > 1500) {
    inter++;
  }  if (sensorValues2[6] > 1500) {
    inter++;
  }  if (sensorValues[1] > 1500) {
    inter++;
  }  if (sensorValues2[5] > 1500) {
    inter++;
  }  if (sensorValues[2] > 1500) {
    inter++;
  }  if (sensorValues2[4] > 1500) {
    inter++;
  }  if (sensorValues[3] > 1500) {
    inter++;
  }  if (sensorValues2[3] > 1500) {
    inter++;
  }  if (sensorValues[4] > 1500) {
    inter++;
  }  if (sensorValues2[2] > 1500) {
    inter++;
  } if (sensorValues[5] > 1500) {
    inter++;
  }  if (sensorValues2[1] > 1500) {
    inter++;
  }  if (sensorValues[6] > 1500) {
    inter++;
  }
}
