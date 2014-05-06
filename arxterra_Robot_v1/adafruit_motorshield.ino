// --------------------------------------------------
// Adafruit Motorshield
// --------------------------------------------------
#include <AFMotor.h> 

AF_DCMotor motorL(1, MOTOR12_64KHZ);
AF_DCMotor motorR(2, MOTOR12_64KHZ);

void move(uint8_t * motordata)
{
  motorL.setSpeed(motordata[2]);   
  motorR.setSpeed(motordata[4]);
  motorL.run(motordata[1]);
  motorR.run(motordata[3]);
}

void safeRover()
{
   motorL.run(RELEASE);
   motorR.run(RELEASE);
   // Serial.println("Safe Rover");
}

void stopMotors(){
}
