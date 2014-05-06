#if pinger == TRUE
// --------------------------------------------------
// Ultrasonic Sensors
// --------------------------------------------------

/* The following three libraries must be downloaded and/or copied */
#include <NewPing.h>

#define MAX_DISTANCE     200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// Instantiate Sensors 
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);    // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int  pingSpeed = 50;     // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;          // Holds the next ping time.
uint16_t usRangeLeft=0;           // ultra-sonic (us) range left 
uint16_t usRangeRight=0;          // ultra-sonic (us) range right

boolean checkSonar()
{  
  // Send ping, get ping time in microseconds (uS) and divide by conversion constant  
  // Serial.print("Ping: ");
  usRangeLeft  = sonarLeft.ping() / US_ROUNDTRIP_CM;
  usRangeRight = sonarRight.ping() / US_ROUNDTRIP_CM;
  // check for ping 
  if (usRangeLeft == 0) usRangeLeft = 200; // 0 = outside set distance range, no ping echo
  if (usRangeRight == 0) usRangeRight = 200;

  //Serial.print(usRangeLeft); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  //Serial.print(" , ");
  //Serial.print(usRangeRight); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  //Serial.println("cm");
  return (usRangeLeft < 40 || usRangeRight < 40);
}

// getters
uint16_t getRangeLeft()
{
  return (usRangeLeft);
}

uint16_t getRangeRight()
{
  return (usRangeRight);
}
#endif
