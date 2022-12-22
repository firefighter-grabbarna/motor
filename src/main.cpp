// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// Det var en gång, och den var grusad.

#include "main.h"

#define ONE_SECOND_IN_MS 1000

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
   Calculates the speed all wheels needed for the vehicle to drive
   in a specified direction.
    - forwardSpeed positive forward
    - sidewaysSpeed positive left
    - rotationspeed positive counter-clockwise
*/
void getWheelSpeeds(int forwardSpeed, int sidewaysSpeed, int rotationSpeed, int (&speedVector)[4])
{
   speedVector[LEFT_FRONT_WHEEL - 1] = min(255, (forwardSpeed + rotationSpeed + sidewaysSpeed));  // 1 in coppa // left front
   speedVector[LEFT_BACK_WHEEL - 1] = min(255, (forwardSpeed + rotationSpeed - sidewaysSpeed));   // 2 in coppa // left back
   speedVector[RIGHT_BACK_WHEEL - 1] = min(255, (forwardSpeed - rotationSpeed + sidewaysSpeed));  // 3 in coppa // right back  ?
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
   Sets the speed of each wheel to the elements of speed_vect.
   -1 because wheel index starts an 1, not 0.
*/
void setWheelSpeed(int (&speed_vect)[4])
{
   setSpeedAndDirection(speed_vect[LEFT_FRONT_WHEEL - 1], LeftFrontWheel);
   setSpeedAndDirection(speed_vect[RIGHT_BACK_WHEEL - 1], RightBackWheel);
   setSpeedAndDirection(speed_vect[RIGHT_FRONT_WHEEL - 1], RightFrontWheel);
   setSpeedAndDirection(speed_vect[LEFT_BACK_WHEEL - 1], LeftBackWheel);
}

/*
   sampleTime is in millisecond
*/
void regulateWheel(unsigned long sampleTime, int& speed, int encoder, int gainFactor){
   int digitalReadVal = 0;
   int oldDigitalReadVal = 0;
   int ticks = 0;
   unsigned long startTime = millis(); 
   while((millis() - startTime) < sampleTime){
      digitalReadVal = digitalRead(encoder);
      if (digitalReadVal != oldDigitalReadVal){ // Only count each gap once
         ticks += digitalReadVal;
      }
      oldDigitalReadVal = digitalReadVal;
   }
   int expectedTicks = 40; // TODO find real value
   int error = gainFactor * (ticks - expectedTicks);
   Serial.print("Ticks: ");
   Serial.println(ticks);
   Serial.print("Speed before: ");
   Serial.println(speed);
   speed -= error;
   Serial.print("Speed after: ");
   Serial.println(speed);
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
   for (int i = 0; i < 4; i++){
      regulateWheel(100, speedVector[i], 0, 1.1);
   }
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

/*
Calculates the angular speed of a wheel using the corresponding encoder.

https://www.motioncontroltips.com/how-are-encoders-used-for-speed-measurement/
ω = 2πn/Nt
ω = Angular speed
n = number of pulses
t = sampling period
N = pulses per rotation
*/
double getAngularSpeed(int wheel){
   int encoder_pin = 2; //get_encoder_pin(wheel);

   // Count number of pulses in one second
   int digitalReadVal = 0;
   int oldDigitalReadVal = 0;
   int n = 0;
   int startTime = millis(); 
   while( (millis() - startTime) < (ONE_SECOND_IN_MS)){
      digitalReadVal = digitalRead(encoder_pin);
      if (digitalReadVal != oldDigitalReadVal){ // Only count each gap once
         n += digitalReadVal;
      }
      oldDigitalReadVal = digitalReadVal;
      //delay(10);  
      // Delay to prevent bouncing, should be short enough to not affect anything else.
      // Max speed is 240 RPM = 25 rad/s
      // ω = 2πn/Nt    =>    n = ωNt/2π    =>    n = 80 when ω = 25
      // 1000 ms / 80 pulses => 1 pulse every 12.5 ms
   }

   // Calculate speed
   int t = 1, N = 20;
   double omega = ( 2 * PI * n ) / ( N * t );
   return omega;
}

/*
   Calculates the RPM of a wheel using the corresponding encoder.
*/
int getRPM(int motor){
   int ang_speed = getAngularSpeed(motor);
   return 2 * PI * ang_speed / 60;
}

int forwardSpeed = 0, sidewaysSpeed = 0, rotationSpeed = 0;
int encoder = -1;

void loop(){
   unsigned long sampleTime = 100;
   double gainFactor = 1.1;
   int response[3];
   int speedVector[4];

   // PID next wheel
   encoder = (encoder + 1) % 4;

   // Read new command
   if (Serial.available()){
      listen(response);
      forwardSpeed = response[0], sidewaysSpeed = response[1], rotationSpeed = response[2];
      getWheelSpeeds(forwardSpeed, sidewaysSpeed, rotationSpeed, speedVector);
      setWheelSpeed(speedVector);
   }

   int digitalReadVal = 0;
   int oldDigitalReadVal = 0;
   int ticks = 0;
   unsigned long startTime = millis(); 
   while((millis() - startTime) < sampleTime){
      digitalReadVal = digitalRead(encoder);
      if (digitalReadVal != oldDigitalReadVal){ // Only count each gap once
         ticks += digitalReadVal;
      }
      oldDigitalReadVal = digitalReadVal;
   }
   int expectedTicks = 40; // TODO find real value
   int error = gainFactor * (ticks - expectedTicks);
   Serial.print("Ticks: ");
   Serial.println(ticks);
   Serial.print("Speed before: ");
   Serial.println(speedVector[encoder]);
   speedVector[encoder] -= error;
   setWheelSpeed(speedVector);
   Serial.print("Speed after: ");
   Serial.println(speedVector[encoder]);
}

void motorStop(){
   LeftFrontWheel.run(RELEASE);
   LeftBackWheel.run(RELEASE);
   RightFrontWheel.run(RELEASE);
   RightBackWheel.run(RELEASE);
}
