// --------------------------------------------------
// Pololu Motor Shield
// --------------------------------------------------

/***********************************
* motion command
* motordata[1]    left run    (FORWARD, BACKWARD, BRAKE, RELEASE)
* motordata[2]    left speed  0 - 255
* motordata[3];   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
* motordata[4];   right speed 0 - 255
***********************************/

#include <DualVNH5019MotorShield.h>

// Instantiate Motor
DualVNH5019MotorShield motor;

// Motor Variables and Default Values
int m1Speed = 0;                 // left motor
int m2Speed = 0;                 // right motor

// Translation Array for Pololu Motor Shield 
int dirArray[5] = {0, 1,-1,0,0};  // forward = index 1, backward = index 2, ...

void move(uint8_t * motordata)
{
       m1Speed = dirArray[motordata[1]] * motordata[2]; // left motor
       m2Speed = dirArray[motordata[3]] * motordata[4]; // right motor
       motor.setSpeeds(m1Speed, m2Speed);

      // motor.setM1Speed(dirArray[motordata[1]] * motordata[2]);
      // motor.setM2Speed(dirArray[motordata[3]] * motordata[4]);
}

void stopMotors() {
  Serial.println("Motors Stopped");
  m1Speed = 0;                  // left motor off
  m2Speed = 0;                  // right motor off
  motor.setSpeeds(m1Speed, m2Speed);
}
