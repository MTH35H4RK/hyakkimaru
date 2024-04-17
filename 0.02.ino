#include <QTRSensors.h>

const int MOTOR_A_ENA = 10;
const int MOTOR_A_IN1 = 8;
const int MOTOR_A_IN2 = 9;


const int MOTOR_B_ENB = 2;
const int MOTOR_B_IN3 = 4;
const int MOTOR_B_IN4 = 3;



const int BASE_SPEED = 255;
const int MAX_SPEED = 255;

//260/7000=0.0371428571428571

float Kp = 0.25;
float Ki = 0.01;
float Kd = 1;

int lastError = 0;
int integral = 0;

void setMotors(int speedA, int speedB) {
  if (speedA < 5 && speedA >= 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_ENA, 0);
  }
  if (speedB < 5 && speedB >= 0) {
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, HIGH);
    analogWrite(MOTOR_B_ENB, 0);
  }
  if (speedA < 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_ENA, -speedA);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_ENA, speedA);
  }


  if (speedB < 0) {
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
    analogWrite(MOTOR_B_ENB, -speedB);

  } else {
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
    analogWrite(MOTOR_B_ENB, speedB);

  }
}


QTRSensors qtr;
QTRSensors qtr2;

const uint8_t SensorCount = 8;
const uint8_t SensorCount2 = 8;

uint16_t sensorValues[SensorCount];
uint16_t sensorValues2[SensorCount2];


void setup()
{
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);


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


  Serial.begin(9600);


  delay(500);
  Serial.print("Start");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 110; i++)
  {
    qtr.calibrate();
    qtr2.calibrate();
    if (i > 20) {
      digitalWrite(52, HIGH);
    }
    if (i > 40) {
      digitalWrite(50, HIGH);
    }
    if (i > 60) {
      digitalWrite(48, HIGH);
    } if (i > 80) {
      digitalWrite(46, HIGH);
    }
    if (i > 100) {
      digitalWrite(44, HIGH);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.print("end");

  //  for (uint8_t i = 0; i < SensorCount; i++)
  //  {
  //    Serial.print(qtr.calibrationOn.minimum[i]);
  //    Serial.print(' ');
  //    Serial.print(qtr2.calibrationOn.minimum[i]);
  //
  //  }
  //  Serial.println();
  //
  //
  //  for (uint8_t i = 0; i < SensorCount; i++)
  //  {
  //    Serial.print(qtr.calibrationOn.maximum[i]);
  //    Serial.print(qtr2.calibrationOn.maximum[i]);
  //
  //    Serial.print(' ');
  //  }
  Serial.println();
  Serial.println();
}
uint16_t position;
uint16_t position1;
uint16_t position2;

void pid() {

  position1 = qtr.readLineBlack(sensorValues);
  position2 = qtr2.readLineBlack(sensorValues2);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //    Serial.print(sensorValues[i]);
    //    Serial.print('\t');

    if (sensorValues[i] > 700)
    {
      position = i * 1000;
    }
  }
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //    Serial.print(sensorValues2[i]);
    //    Serial.print('\t');

    if (sensorValues2[i] > 700)
    {
      position = (i * 1000) + 7000;
    }
  }


  //
  //Serial.print(position1);
  //Serial.print("  ");
  //Serial.print(position2);
  //Serial.print("  ");
  Serial.println(position);

  int error = position - 7500;
  integral += error;

  integral = constrain(integral, -10000, 10000);

  int derivative = error - lastError;

  int speedDifference = Kp * error + Ki * integral + Kd * derivative;

  int speedA = BASE_SPEED + speedDifference; // Right
  int speedB = BASE_SPEED - speedDifference; // Left


  speedA = constrain(speedA, -87, MAX_SPEED);
  speedB = constrain(speedB, -87, MAX_SPEED);
  //  Serial.print("A:");
  //  Serial.print(speedA);
  //  Serial.print("   B:");
  //  Serial.println(speedB);

  setMotors(speedA, speedB);

  lastError = error;
}
void loop() {
  pid();
}
