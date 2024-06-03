#include <util/atomic.h>
#include <math.h>



#include "Dictionary.h"
const int maxDigits = 25;  // Adjust this according to the maximum number of digits you expect
int numbers[maxDigits];
int degrees[maxDigits];
int numbersLength = 0;
float bigForward = 0.385;
float smallForward = 0.288;
float rotate90 = 0.137 ;
float rotate180 = 0.134 * 2 ;
float rotate45 = 0.135 / 2 ;
float rotate135 = rotate45 + rotate90;
int travelTime[maxDigits * 2];
float velocities[maxDigits * 2][2];
Dictionary<int, float> m;
// Map direction numbers to degrees
int directionDegrees[] = {0, 0, 45, 90, 135, 180, -135, -90, -45};
int current_angle = 0;







bool MOVE = true;

class SimplePID {
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;

  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(150), eprev(0.0), eintegral(0.0) {}

    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
      int e = target - value;
      float dedt = (e - eprev) / (deltaT);
      eintegral = eintegral + e * deltaT;
      float u = kp * e + kd * dedt + ki * eintegral;
      pwr = (int) fabs(u);
      if ( pwr > umax ) {
        pwr = umax;
      }
      dir = 1;
      if (u < 0) {
        dir = -1;
      }
      eprev = e;
    }
};

#define NMOTORS 2
#define M0 0
#define M1 1


const int enca[] = { 3, 2} ;
const int encb[] = { 11, 12};
const int pwm[] = { 10, 5};
const int in1[] = { 8, 7};
const int in2[] = { 9, 6};

long prevT = 0;
int posPrev[] = {0, 0};

volatile int posi[] = {0, 0};

SimplePID pid[NMOTORS];

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  }
  else {
    posi[j]--;
  }
}

float target_f[] = {0, 0};
long target[] = {0, 0};
int tot = 0;

void setTarget(float t, float deltaT) {
  float positionChange[2] = {0.0, 0.0};
  float pulsesPerMeter = 350 * 7.402;
  float accumulatedTime = 0.0;

  for (int i = 0; i < maxDigits * 2; i++) {
    accumulatedTime += travelTime[i];
    if (t < accumulatedTime) {
      positionChange[0] = velocities[i][0] * deltaT * pulsesPerMeter;
      positionChange[1] = velocities[i][1] * deltaT * pulsesPerMeter;
      break;
    }
  }

  if (t >= accumulatedTime) {
    MOVE = false;
  }

  // Update targets
  for (int k = 0; k < 2; k++) {
    target_f[k] += positionChange[k];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
}

long St = 0;

void moove() {

  if (St == 0) {
    St = micros(); // Store the start time on the first call
  }

  long currT = micros() - St;

  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  setTarget(currT / 1.0e6, deltaT);
  long pos[2];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }
  for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
    setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
  }
  for (int i = 0; i < 2; i++) {
    Serial.print(target[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < 2; i++) {
    Serial.print(pos[i]);
    Serial.print(" ");
  }
  Serial.println();
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
  Serial.print(m[45]);
  Serial.print(m[90]);
  Serial.print(m[135]);
  Serial.print(m[180]);
  Serial.print(m[-45]);
  Serial.print(m[-90]);
  Serial.print(m[-135]);

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
    if(degrees[i]==0)travelTime[j] =0;
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



  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pid[k].setParams(1.8, 0.08, 0.18, 200);
  }
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, RISING);
}


void loop() {
  moove();
}
