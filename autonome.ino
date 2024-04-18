#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0; // specify posi as volatile
long prevTime = 0;
float prevError = 0;
float integral = 0;

float PPR = 105.6;
const int MAX_MOTOR_SPEED = 150;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("target pos");
}

void loop() {
  // Set target position
  float target = 10 * PPR;

  // Calculate PID control signal
  float control = calculatePID(target);

  // Limit control signal
  float power = constrain(fabs(control), 0, MAX_MOTOR_SPEED);

  // Set motor direction
  int dir = control < 0 ? 1 : -1;

  // Control the motor
  setMotor(dir, power);

  // Debugging output
  Serial.print(target);
  Serial.print(" ");
  Serial.print(posi);
  Serial.println();
}

float calculatePID(int target) {
  // PID constants
  float kp = 75;
  float kd = 2.5;
  float ki = 0;

  // Time difference
  long currentTime = micros();
  float deltaTime = (currentTime - prevTime) / 1.0e6;
  prevTime = currentTime;

  // Read position
  int pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // Calculate error
  int error = pos - target;

  // Calculate derivative
  float derivative = (error - prevError) / deltaTime;

  // Calculate integral
  integral += error * deltaTime;

  // Calculate control signal
  float control = kp * error + kd * derivative + ki * integral;

  // Store previous error
  prevError = error;

  return control;
}

void setMotor(int dir, int pwmVal) {
  analogWrite(PWM, pwmVal);
  digitalWrite(IN1, dir == 1 ? HIGH : LOW);
  digitalWrite(IN2, dir == -1 ? HIGH : LOW);
}

void readEncoder() {
  int b = digitalRead(ENCB);
  posi += (b > 0) ? 1 : -1;
}
