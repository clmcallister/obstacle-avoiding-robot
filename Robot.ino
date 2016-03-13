#include <NewPing.h>
#include <AFMotor.h>
#include <Servo.h>

#define PING_PIN 7
#define MAX_DISTANCE 400

#define LEFT_MOTOR_NUM 1
#define RIGHT_MOTOR_NUM 2

#define MAX_MOTOR_SPEED 180

#define OBSTACLE_DIST_CM 23

AF_DCMotor motor1(LEFT_MOTOR_NUM);
AF_DCMotor motor2(RIGHT_MOTOR_NUM);

Servo pingServo;

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE);

boolean isFarObstacle = false;
boolean isCloseObstacle = false;
int robotCenter = 90;

void setup() {
  Serial.begin(9600);

  pingServo.attach(9);
  pingServo.write(robotCenter);
}

void loop() {

  moveRobot(MAX_MOTOR_SPEED, FORWARD);

  unsigned int distance_cm = sonar.ping() / US_ROUNDTRIP_CM;

  Serial.print("Distance from object is: ");
  Serial.print(distance_cm);
  Serial.print("\n");

  if (distance_cm < OBSTACLE_DIST_CM) {

    stopRobot();

    unsigned int left_object_distance_cm = scanLeft();
    unsigned int right_object_distance_cm = scanRight();

    if (left_object_distance_cm < OBSTACLE_DIST_CM && right_object_distance_cm < OBSTACLE_DIST_CM) {

      // The area is blocked so turn around
      moveRobot(MAX_MOTOR_SPEED, BACKWARD);
      delay(1000);
      stopRobot();
      turnRobotLeft(MAX_MOTOR_SPEED, 5000);
      moveRobot(MAX_MOTOR_SPEED, FORWARD);

    } else if (left_object_distance_cm < right_object_distance_cm && right_object_distance_cm >= OBSTACLE_DIST_CM) {

      // Obstacle on the left and we have a better chance by going right
      turnRobotRight(MAX_MOTOR_SPEED, 1000); // Turn right to avoid object on the left

    } else {

      // Obstacle is on the right side and we have a better chance by going left
      turnRobotLeft(MAX_MOTOR_SPEED, 1000);

    }
    
  }
  
}

void moveRobot(int moveSpeed, int moveDirection) {
  motor1.run(moveDirection);
  motor2.run(moveDirection);
  motor1.setSpeed(moveSpeed);
  motor2.setSpeed(moveSpeed);
}

void stopRobot() {
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

unsigned int scanLeft() {
  unsigned int distance_cm = 0;
  for (int i = robotCenter; i <= 180; i++) {
    pingServo.write(i);
    delay(15);
    distance_cm = sonar.ping() / US_ROUNDTRIP_CM;
  }
  pingServo.write(robotCenter);
  return distance_cm;
}

unsigned int scanRight() {
  unsigned int distance_cm = 0;
  for (int i = robotCenter; i >= 0; i--) {
    pingServo.write(i);
    delay(15);
    distance_cm = sonar.ping() / US_ROUNDTRIP_CM;
  }
  pingServo.write(robotCenter);
  return distance_cm;
}
