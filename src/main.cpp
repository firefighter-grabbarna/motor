
#include <AFMotor.h>
#include <Arduino.h>
#include "pid.hpp"

//const int PWM_PIN = A0;
const int BAUD_RATE = 9600;

const int RIGHT_FRONT_WHEEL = 1;
const int LEFT_BACK_WHEEL = 0;
const int RIGHT_BACK_WHEEL = 2;
const int LEFT_FRONT_WHEEL = 3;

AF_DCMotor RightFrontWheel(RIGHT_FRONT_WHEEL + 1);
AF_DCMotor LeftBackWheel(LEFT_BACK_WHEEL + 1);
AF_DCMotor RightBackWheel(RIGHT_BACK_WHEEL + 1);
AF_DCMotor LeftFrontWheel(LEFT_FRONT_WHEEL + 1);

void setup(){
   Serial.begin(BAUD_RATE);

   // turn on motor dont crash code
   RightBackWheel.run(RELEASE);
   LeftBackWheel.run(RELEASE);
   LeftFrontWheel.run(RELEASE);
   RightFrontWheel.run(RELEASE);
}

/*
   Calculates the speed all wheels needed for the vehicle to drive
   in a specified direction.
    - forwardSpeed positive forward
    - sidewaysSpeed positive left
    - rotationspeed positive counter-clockwise
*/
void calcWheelSpeeds(int forwardSpeed, int sidewaysSpeed, int rotationSpeed, int (&speedVector)[4]){
   speedVector[LEFT_FRONT_WHEEL] = min(255, (forwardSpeed + rotationSpeed + sidewaysSpeed));  // 1 in coppa // left front
   speedVector[LEFT_BACK_WHEEL] = min(255, (forwardSpeed + rotationSpeed - sidewaysSpeed));   // 2 in coppa // left back
   speedVector[RIGHT_BACK_WHEEL] = min(255, (forwardSpeed - rotationSpeed + sidewaysSpeed));  // 3 in coppa // right back  ?
   speedVector[RIGHT_FRONT_WHEEL] = min(255, (forwardSpeed - rotationSpeed - sidewaysSpeed)); // 4 in coppa // right front ?
}

/*
   Given a speed and a wheel sets the correct direction for the wheel
    - A negative speed means backwards
    - A positive speed means fortwads
*/
void setSpeedAndDirection(int speed, AF_DCMotor &wheel){
   wheel.setSpeed(min(255, abs((int)speed)));
   if (speed >= 0){
      wheel.run(FORWARD);
   }
   else if (speed <= 0){
      wheel.run(BACKWARD);
   }
}

/*
   Sets the speed of each wheel to the elements of speed_vect.
*/
void setWheelSpeed(int (&speed_vect)[4]){
   setSpeedAndDirection(speed_vect[LEFT_FRONT_WHEEL], LeftFrontWheel);
   setSpeedAndDirection(speed_vect[RIGHT_BACK_WHEEL], RightBackWheel);
   setSpeedAndDirection(speed_vect[RIGHT_FRONT_WHEEL], RightFrontWheel);
   setSpeedAndDirection(speed_vect[LEFT_BACK_WHEEL], LeftBackWheel);
}

/*
   Calculates the speed of each wheel and then runs.
    - forwardSpeed positive forward
    - sidewaysSpeed positive left
    - rotationspeed positive counter-clockwise
*/
void runWheels(int forwardSpeed, int sidewaysSpeed, int rotationSpeed){
   int speedVector[4] = {0, 0, 0, 0};
   calcWheelSpeeds(forwardSpeed, sidewaysSpeed, rotationSpeed, speedVector);
   setWheelSpeed(speedVector);
}

/*
   Waits for input from serial and then updates response[] with said input
*/
void listen(int (&response)[3]){
   while (true) {
      while(!(Serial.available())){
         ;
      }

      String input = Serial.readStringUntil('\n');
      String cpy = input;
      cpy.trim();
      if (cpy == "SCIP2.0") {
         Serial.println("MOTOR");
         Serial.println("");
         continue;
      } else if (cpy == "") {
         continue;
      }
      char myArray[input.length() + 1];        //as 1 char space for null is also required
      strcpy(myArray, input.c_str());    
   
      char * token = strtok(myArray, " ");   // Extract the first token

      // loop through the string to extract all other tokens
      for(int i = 0; token != NULL; i++){
         response[i] = atoi(token);
         token = strtok(NULL, " ");
      }
      break;
   }
}


int getEncoderPin(int id) {
   if (id == RIGHT_BACK_WHEEL) {
      return A0;
   } else if (id == RIGHT_FRONT_WHEEL) {
      return A1;
   } else if (id == LEFT_BACK_WHEEL) {
      return A2;
   } else if (id == LEFT_FRONT_WHEEL) {
      return A3;
   }
   return -1;
}

int encoder = -1;
float totalError = 0;
float timeSinceLastIter[] = {0, 0, 0, 0};
//float iterationTime = 0.1;
int targetSpeedVector[4] = {0, 0, 0, 0};
int currSpeedVector[4] = {0, 0, 0, 0};
int forwardSpeed = 0, sidewaysSpeed = 0, rotationSpeed = 0;
double Kp = 0.2;
double Ki = 0.2;
int response[3];

PID pids[4] = {PID(), PID(), PID(), PID()};

void loop(){
   // PID next wheel
   encoder = (encoder + 1) % 4;
   //encoder = 0;

   if (Serial.available()){
      listen(response);
      forwardSpeed = response[0], sidewaysSpeed = response[1], rotationSpeed = response[2];
      totalError = 0;
      calcWheelSpeeds(forwardSpeed, sidewaysSpeed, rotationSpeed, targetSpeedVector);
      for (int i = 0 ; i < 4; ++i ) {
         pids[i].set_target_speed(targetSpeedVector[i]);

      }
   }

   int digitalRead_val = analogRead(getEncoderPin(encoder)) > 500;
   currSpeedVector[encoder] = pids[encoder].update(digitalRead_val);
   // unsigned long start_time = micros();

   // while ((analogRead(getEncoderPin(encoder)) > 500) == digitalRead_val) {
   //    if (micros() > start_time + 300000) break;
   // }
   // long time1 = micros() - start_time;
   // while ((analogRead(getEncoderPin(encoder)) > 500) != digitalRead_val) {
   //    if (micros() > start_time + 300000) break;
   // }
   // long time2 = micros() - start_time;

   // float ticks_per_second = 1e6/((float)(time2-time1));
   // if (time2 > 300000) ticks_per_second = 0;

   // float lastIter = timeSinceLastIter[encoder];
   // float now = micros();
   // timeSinceLastIter[encoder] = now;
   // float timeSinceLast = now - lastIter;

   // float expectedTicks = (targetSpeedVector[encoder]);
   // int error = ticks_per_second - expectedTicks;
   // totalError += error * timeSinceLast;

   // //Serial.print("=== Encoder: "); Serial.print(encoder); Serial.println(" ===");
   // //Serial.print("Ticks/s: ");
   // Serial.println(ticks_per_second);
   
   //currSpeedVector[encoder] -= Kp * error + Ki * totalError;

   setWheelSpeed(currSpeedVector);
}


