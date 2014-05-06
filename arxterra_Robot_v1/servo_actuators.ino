// --------------------------------------------------
// Ultrasonic Sensors
// --------------------------------------------------

/* The following library must be downloaded and/or copied */
#include <Servo.h> 

// Set Servo Default Limits       *** In the future these will be read from EEPROM ***
const int servoMin       =  544;  // microseconds,   0 degress (Arduino default value)
const int servoMax       = 2400;  // microseconds, 180 degress (Arduino default value)
const int servoSpeed     =   15;  // milliseconds, experimentally determined
const int panLeftLimit   =    0;  // degress
const int panReset       =    0;  // degrees
const int panHome        =   90;  // degrees
const int panRightLimit  =  180;  // degrees
const int tiltDownLimit  =    0;  // degress
const int tiltReset      =   90;
const int tiltHome       =   90;
const int tiltUpLimit    =  180;  // degress

// *** RESET VALUES *** 

int panPosition;        // change to int panPosition = panHome; if using commented out code in move_camera function
int tiltPosition;      // change to int tiltPosition = tiltUpLimit; if using commented out code in move_camera function



// Instantiate Devices 
Servo panServo;
Servo tiltServo;

void init_servos()
{
  panServo.attach(SERVO_PIN_PAN, servoMin, servoMax);
  tiltServo.attach(SERVO_PIN_TILT, servoMin, servoMax);
 
   
  // reset android device 
  reset_camera;
}

// Need to conver to Interrupt driven servo routine see this link, Arduino examples, and millis() for ideas
// http://scolton.blogspot.com/2010/07/arduino-interrupt-driven-servo-routine.html
// The varSpeedServo library created to control speed the speed of a servo linked to here
// http://forum.arduino.cc/index.php?PHPSESSID=1m5s2c18fejtuos7t8qtiqidl0&topic=6031.0
// in post 14 is no longer found in the repository.
// this method seems to be the one you find in the most places
// http://web.cecs.pdx.edu/~gerry/class/EAS199A/notes/10/servos_with_Arduino_PSU_2011_slides.pdf

void move_camera(uint8_t panAngle, uint8_t tiltAngle){
/*
  // limit check
  constrain(panAngle, panLeftLimit, panRightLimit);
  constrain(tiltAngle, tiltDownLimit, tiltUpLimit);
  
  // find maximum number of steps
  int panSteps = abs(panPosition - panAngle);
  int tiltSteps = abs(tiltPosition - tiltAngle);
  int steps = max(panSteps, tiltSteps); 

  for (int i = 1; i <= steps; i++)
  {
      // calculate next step 
      if (panPosition < panAngle) panPosition++;
      else if (panPosition > panAngle) panPosition--;  
      if (tiltPosition < tiltAngle) tiltPosition++;
      else if (tiltPosition > tiltAngle) tiltPosition--;
  
      // tell servos to take the step
      panServo.write(panPosition);
      tiltServo.write(tiltPosition);
    
      // wait 15ms (default) for the servo to reach the position 
      delay(servoSpeed);     
 }
 */ 
      
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
}

void reset_camera()
{
  panPosition = panReset;       // pan servo position at reset
  panServo.write(panReset);
  delay(1000);
  tiltPosition = tiltReset;     // tilt servo position at reset
  tiltServo.write(tiltReset);
  delay(1000); 
}

void home_camera()
{
  move_camera(panHome,tiltHome);
}

// getters

byte getPanPosition()
{
  return byte(panPosition);
}

byte getTiltPosition()
{
  return byte(abs(tiltPosition-180));  //***********************************************
}

  
/* note: If needed send end-of-pan/tilt message allowing rover's orientation
 *       display to be updated. Because we are locking the playback head this
 *       may or may not required... does Android send telemetry independent of
 *       Arduino?
 */
