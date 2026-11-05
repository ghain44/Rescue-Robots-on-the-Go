#include <Servo.h>

// ================= PIN CONFIG =================
const int L_PWM = 5;
const int L_DIR = 7;
const int R_PWM = 6;
const int R_DIR = 8;

const int TRIG_PIN = A5;
const int ECHO_PIN = A4;
const int SERVO_PIN = 3;

const int IR_LEFT_PIN  = A0;
const int IR_RIGHT_PIN = A1;

const int  HEAT_PIN = A3;
const int  HEAT_THRESHOLD = 400;
const bool HEAT_ACTIVE_LOW = true;

// ================= SETTINGS =================
const int SPEED        = 150;
const int TURN_SPEED   = 130;
const int OBSTACLE_CM  = 25;   // stop closer than this (raised a bit for braking distance)
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 160;
const int SERVO_RIGHT  = 20;
const int MAX_AVOID_TRIES = 4;

Servo scanServo;

// ================= MOTOR HELPERS =================
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(L_DIR, leftSpeed  >= 0 ? HIGH : LOW);
  digitalWrite(R_DIR, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(L_PWM, abs(leftSpeed));
  analogWrite(R_PWM, abs(rightSpeed));
}

void forward()   { setMotors(SPEED, SPEED); }
void backward()  { setMotors(-SPEED, -SPEED); }
void turnLeft()  { setMotors(-TURN_SPEED, TURN_SPEED); }
void turnRight() { setMotors(TURN_SPEED, -TURN_SPEED); }
void stopCar()   { setMotors(0, 0); }

// ================= SENSORS =================
long readDistanceOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 400;           // no echo = nothing in range
  return (long)(duration * 0.0343 / 2);
}

// Median of 3 readings: filters random noise and missed echoes
long readDistanceCm() {
  long a = readDistanceOnce();
  delay(5);
  long b = readDistanceOnce();
  delay(5);
  long c = readDistanceOnce();

  if (a > b) { long t = a; a = b; b = t; }
  if (b > c) { long t = b; b = c; c = t; }
  if (a > b) { long t = a; a = b; b = t; }
  return b;
}

long distanceAt(int angle) {
  scanServo.write(angle);
  delay(350);
  return readDistanceCm();
}

bool irLeftDetected()  { return digitalRead(IR_LEFT_PIN)  == LOW; }
bool irRightDetected() { return digitalRead(IR_RIGHT_PIN) == LOW; }

bool heatDetected() {
  int v = analogRead(HEAT_PIN);
  return HEAT_ACTIVE_LOW ? (v < HEAT_THRESHOLD) : (v > HEAT_THRESHOLD);
}

// True if ANY sensor sees something in front
bool obstacleDetected() {
  if (irLeftDetected() || irRightDetected()) return true;
  long d = readDistanceCm();
  return (d > 0 && d < OBSTACLE_CM);
}

// ================= BEHAVIORS =================
void avoidHeat() {
  stopCar();
  Serial.println("ALERT: Heat detected!");
  delay(300);
  backward();
  delay(500);
  turnRight();
  delay(600);
  stopCar();
}

void avoidObstacle() {
  stopCar();                                  // STOP FIRST
  Serial.println("Obstacle detected - stopped");
  delay(150);

  for (int attempt = 0; attempt < MAX_AVOID_TRIES; attempt++) {
    // Back away a little
    backward();
    delay(250);
    stopCar();

    // Scan both sides
    long leftDist  = distanceAt(SERVO_LEFT);
    long rightDist = distanceAt(SERVO_RIGHT);
    scanServo.write(SERVO_CENTER);
    delay(200);

    // Both sides blocked -> turn around
    if (leftDist < OBSTACLE_CM && rightDist < OBSTACLE_CM) {
      Serial.println("Boxed in - turning around");
      turnRight();
      delay(900);
      stopCar();
    } else if (leftDist > rightDist) {
      turnLeft();
      delay(450);
      stopCar();
    } else {
      turnRight();
      delay(450);
      stopCar();
    }

    delay(100);

    // Only continue if the way ahead is actually clear
    if (!obstacleDetected()) {
      Serial.println("Path clear");
      return;
    }
  }

  // Still blocked after all tries: stay stopped and report
  stopCar();
  Serial.println("Blocked - waiting");
  delay(1000);
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(9600);

  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(HEAT_PIN, INPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);
  delay(500);

  Serial.println("Robot ready");
}

void loop() {
  // 1. Heat has top priority
  if (heatDetected()) {
    avoidHeat();
    return;
  }

  // 2. Any object (IR or ultrasonic): stop, then avoid
  if (obstacleDetected()) {
    avoidObstacle();
    return;
  }

  // 3. Path clear
  forward();
}