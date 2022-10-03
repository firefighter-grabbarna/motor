// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// Det var en gång, och den var grusad.

#include "main.h"

void setup()
{
   // 115200
   Serial.begin(BAUD_RATE);
   //Serial.println("Motor test!");

   // turn on motor dont crash code
   RightBackWheel.run(RELEASE);
   LeftBackWheel.run(RELEASE);
   LeftFrontWheel.run(RELEASE);
   RightFrontWheel.run(RELEASE);
}

/*
   Calculates the speed all wheels need for the vehicle to drive
   in a specified direction.
    - forwardSpeed positive forward
    - sidewaysSpeed positive left
    - rotationspeed positive counter-clockwise
*/
void getWheelSpeeds(int forwardSpeed, int sidewaysSpeed, int rotationSpeed, int (&speedVector)[4])
{
   speedVector[LEFT_FRONT_WHEEL - 1] = min(255, (forwardSpeed + rotationSpeed + sidewaysSpeed));  // 1 in coppa // left front
   speedVector[LEFT_BACK_WHEEL - 1] = min(255, (forwardSpeed - rotationSpeed + sidewaysSpeed));   // 2 in coppa // left back
   speedVector[RIGHT_BACK_WHEEL - 1] = min(255, (forwardSpeed + rotationSpeed - sidewaysSpeed));  // 3 in coppa // right back  ?
   speedVector[RIGHT_FRONT_WHEEL - 1] = min(255, (forwardSpeed - rotationSpeed - sidewaysSpeed)); // 4 in coppa // right front ?
}

/*
   Given a speed and a wheel
   sets the correct direction for the wheel

   A negative speed means backwards
   A positive speed means fortwads
*/
void setSpeedAndDirection(int speed, AF_DCMotor &wheel)
{
   wheel.setSpeed(min(255, abs((int)speed)));
   if (speed >= SPEED_THRESHOLD)
   {
      wheel.run(FORWARD);
   }
   else if (speed <= SPEED_THRESHOLD)
   {
      wheel.run(BACKWARD);
   }
}

/*
   Sets the speed of each wheel to the elements of speed_vect
*/
void setWheelSpeed(int (&speed_vect)[4])
{
   setSpeedAndDirection(speed_vect[LEFT_FRONT_WHEEL - 1], LeftFrontWheel);
   setSpeedAndDirection(speed_vect[RIGHT_BACK_WHEEL - 1], RightBackWheel);
   setSpeedAndDirection(speed_vect[RIGHT_FRONT_WHEEL - 1], RightFrontWheel);
   setSpeedAndDirection(speed_vect[LEFT_BACK_WHEEL - 1], LeftBackWheel);
}

/*
   Calculates the speed of each wheel and then runs.
    - forwardSpeed positive forward
    - sidewaysSpeed positive left
    - rotationspeed positive counter-clockwise
*/
void runWheels(int forwardSpeed, int sidewaysSpeed, int rotationSpeed){
   int speedVector[4];
   getWheelSpeeds(forwardSpeed, sidewaysSpeed, rotationSpeed, speedVector);
   setWheelSpeed(speedVector);
}

// Waits for input from serial and then updates response[] with said input
void listen(int (&response)[3]){
   while (true) {
      while(!(Serial.available())){
         ; // Busy wait mycket effektivt
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

void loop()
{
   int response[3];
   listen(response);
   runWheels(response[0], response[1], response[2]);
}
void motorStop()
{
   LeftFrontWheel.run(RELEASE);
   LeftBackWheel.run(RELEASE);

   RightFrontWheel.run(RELEASE);
   RightBackWheel.run(RELEASE);
}
