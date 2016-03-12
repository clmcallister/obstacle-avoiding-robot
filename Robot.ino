#include <NewPing.h>
#include <AFMotor.h>

#define PING_PIN 7
#define MAX_DISTANCE 400

#define LEFT_MOTOR_NUM 1
#define RIGHT_MOTOR_NUM 2

#define MAX_MOTOR_SPEED 160

#define FAR_OBSTACLE_DIST_CM 30
#define NEAR_OBSTACLE_DIST_CM 15

AF_DCMotor motor1(LEFT_MOTOR_NUM);
AF_DCMotor motor2(RIGHT_MOTOR_NUM);

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE);

boolean isFarObstacle = false;
boolean isCloseObstacle = false;

void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned int distance_cm = sonar.ping() / US_ROUNDTRIP_CM;

  Serial.print("Distance from object is: ");
  Serial.print(distance_cm);
  Serial.print("\n");

  if (distance_cm <= FAR_OBSTACLE_DIST_CM && distance_cm >= NEAR_OBSTACLE_DIST_CM) {
    isFarObstacle = true;
    isCloseObstacle = false;
  } else if (distance_cm < NEAR_OBSTACLE_DIST_CM) {
    isFarObstacle = false;
    isCloseObstacle = true;
  } else {
    isFarObstacle = false;
    isCloseObstacle = false;
  }

  if (!isFarObstacle && !isCloseObstacle) {
    Serial.println("No obstacles. Running motors...");
    motor1.run(FORWARD);
    motor1.setSpeed(MAX_MOTOR_SPEED);
    motor2.run(FORWARD);
    motor2.setSpeed(MAX_MOTOR_SPEED);
    delay(500);
    Serial.println("After delay");
  }
  
  if (isFarObstacle) {
    Serial.println("Approaching obstacle. Stopping motors from forward movement.");
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    Serial.println("Turning left");
    turnRobotLeft(MAX_MOTOR_SPEED, 1000);
  }

  if (isCloseObstacle) {
    Serial.println("Very close to obstacle. Stopping motors from forward movement.");
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    delay(100);
    Serial.println("Reversing");
    moveRobot(MAX_MOTOR_SPEED, BACKWARD, 1000);
    Serial.println("Now turning left");
    turnRobotLeft(MAX_MOTOR_SPEED, 1000);
  }
}

void moveRobot(int moveSpeed, int moveDirection, int duration) {
  motor1.run(moveDirection);
  motor2.run(moveDirection);
  motor1.setSpeed(moveSpeed);
  motor2.setSpeed(moveSpeed);
  delay(duration);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

void turnRobotLeft(int turnSpeed, int duration) {
  motor2.run(FORWARD);
  motor2.setSpeed(turnSpeed);
  delay(duration);
  motor2.run(RELEASE);
}

void turnRobotRight(int turnSpeed, int duration) {
  motor1.run(FORWARD);
  motor1.setSpeed(turnSpeed);
  delay(duration);
  motor1.run(RELEASE);
}

