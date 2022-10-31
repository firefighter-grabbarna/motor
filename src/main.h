#ifndef MAIN_H
#define MAIN_H

#include <AFMotor.h>
#include <Arduino.h>

const int PWM_PIN = A0;
const int standStill = 1500; // Controller in middle position
const int BAUD_RATE = 9600;

const int RIGHT_FRONT_WHEEL = 1;
const int LEFT_BACK_WHEEL = 2;
const int RIGHT_BACK_WHEEL = 3;
const int LEFT_FRONT_WHEEL = 4;

const int SPEED_THRESHOLD = 0;

AF_DCMotor RightFrontWheel(1);
AF_DCMotor LeftBackWheel(2);
AF_DCMotor RightBackWheel(3);
AF_DCMotor LeftFrontWheel(4);

void setup();

int convertSpeed(int speed);
void setSpeed(int speed);

void moveForward();
void moveBackward();
void moveSidewaysRight();
void moveSidewaysLeft();

void rotateLeft();
void rotateRight();

void moveRightForward();
void moveRightBackward();
void moveLeftForward();
void moveLeftBackward();

void motorStop();

#endif