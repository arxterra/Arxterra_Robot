/* The following two libraries are included with the Arduino IDE  */  
#include <SPI.h>   // supports SPI serial communication with max3421E chip on Mega ADK
#include <Adb.h>
#include <EEPROM.h>
#include <Wire.h>      // I2C support (currently not implemented)
#include <L3G4200D.h>  // 3-axis Gyro (currently not implemented)

#define FALSE 0
#define TRUE  1

#define otg   TRUE
#define adb   FALSE
#define LiPO  TRUE
#define NiMH  FALSE

/**************** Robot Configuration ***************
 * Set Connection Type/Battery # and Type/ Pinger.  *
 ***************************************************/
#include "pinouts_robot.h" // replace with your pinout (see note)
                           //-- use pinouts_robot.h as template
#define comm otg           // Set it to adb or otg depending on your arduino connection.
#define pinger FALSE       //FALSE if no ultrasonic sensor is used.
#define Digital_Battery TRUE //FALSE if no digital battery is used.
#define Motor_Battery TRUE   //FALSE if no motor battery is used.
#define digital_battery_chem LiPO//Digital battery chemical type(LiPO or NiMH)
#define digital_battery_cells 8  //Digital battery # of cells
#define motor_battery_chem NiMH  //Motor battery chemical type (LiPO or NiMH)
#define motor_battery_cells 7    //Motor battery # of cells
/*
 *  KEEP OUT DO NOT TOUCH BELOW THIS LINE
 */

/******************* Robot Commands ****************
 * source: Arxterra
 ***************************************************/
// Arxterra Commands to Numeric Value Mapping
//               Data[0] =   CMD TYPE | Qual
//                        bit  7654321   0      
#define MOVE         0x01   // 0000000   1        
#define CAMERA_MOVE  0x02   // 0000001   0 
#define CAMERA_RESET 0x05   // 0000010   0
#define CAMERA_HOME  0x04   // 0000010   0
#define READ_EEPROM  0x06   // 0000011   0 
#define WRITE_EEPROM 0x07   // 0000011   1
#define SAFE_ROVER   0x08   // 0000100   0
#define SLEEP        0x0A   // 0000101   0
#define WAKEUP       0x0B   // 0000101   1

/******************* Robot Telemetry ****************
 * source: Arxterra
 ***************************************************/
 // Telemetry Identifiers to Numeric Value Mapping 
#define  MOTOR1_CURRENT_ID  0x01           // motor 1 is left motor
#define  MOTOR2_CURRENT_ID  0x02           // motor 2 is right motor
#define  TEMP_SENSOR_ID     0x03           // temperature sensor
#define  RANGE_LEFT_ID      0x04           // ultrasonic range 1 is left
#define  RANGE_RIGHT_ID     0x05           // ultrasonic range 2 is right
#define  CLEAN_BATTERY_ID   0x06           // Arduino battery
#define  DIRTY_BATTERY_ID   0x07           // DC and servo motors battery 
#define  PAN_POSITION_ID    0x08           // originally defined as pan and tilt
#define  TILT_POSITION_ID   0x09           // not in original definition
#define  EEPROM_RESPONSE_ID 0x0A           // sent in response to EEPROM Read Command


#define  FLAG  -128                        // currently used by fuel gauge

/******************* Robot Sensors ****************
* source: Found in CommunicationRobotPilot Folder
 ***************************************************/
   
uint16_t sensor_value;              // Used for Current Sensor
                                    // last sensor value sent to control panel
                                    // used for comparison with current sensor value
                                    // if different data packet is sent to the control panel
int16_t   batteryDigitalV = 0;
int16_t   batteryMotorsV = 0;
uint16_t  currentM1=0;              // 10 bit right justified value from ADC
uint16_t  currentM2=0;              // 10 bit right justified value from ADC
uint16_t  tempChassis=0;            // 10 bit right justified value from ADC
uint16_t  positionPan=0;
uint16_t  positionTilt=0;

 /**************** Global Variables ****************
* source: Found in CommunicationRobotPilot Folder  *
 ***************************************************/
// EEPROM
uint16_t  eeprom_address;
uint8_t   eeprom_data;

// Timer Variable
unsigned long timer;    // unsigned 32-bit

 /******************* Battery Definitions ****************
 *      LiPO AND NiMH properties are defined here        *
 *********************************************************/

// Battery Voltage Data
// LiPO
double  LiPO_V1 = 4.0;        // volts/cell, 90% point on LiPO curve at 1C
double  LiPO_V0 = 3.4;        // volts/cell, 10% point on LiPO curve at 1C
double  LiPO_Vsafe = 3.2;     // volts/cell, 2.7 volts/cell = discharged voltage 
// NiMH
double  NiMH_V1 = 1.1875;     // volts/cell
double  NiMH_V0 = 0.89;       // volts/cell
double  NiMH_Vsafe = 0.87;    // volts/cell, 0.9 volts/cell = discharged voltage
double d_V1;
double d_V0;
double d_Vsafe;  
#if Digital_Battery == TRUE       //If Using a Digital Battery Check whether it's LiPO or NiMH
                                  //to use correct battery values.
  #if digital_battery_chem == LIPO
    d_V1=LiPO_V1;
    d_V0=LiPO_V0;
    d_Vsafe=LiPO_Vsafe;
  #elif digital_battery_chem == NiMH
    d_V1=NiMH_V1;
    d_V0=NiMH_V0;
    d_Vsafe=NiMH_Vsafe;
  #endif
  // Digital Battery
  double  cells_per_digital_battery = digital_battery_cells; // 2S or 3S
  double  battery_digital_V1 = cells_per_digital_battery * d_V1;       // volts, 90% point on LiPO curve at 1C
  double  battery_digital_V0 = cells_per_digital_battery * d_V0;       // volts, 10% point on LiPO curve at 1C
  double  battery_digital_dV  = battery_digital_V1 - battery_digital_V0;  // difference
  double  battery_digital_Vsafe = cells_per_digital_battery * d_Vsafe; // volts, 2.7 volts/cell = discharged voltage 
#endif

#if Motor_Battery == TRUE         //If Using a Motor Battery Check whether it's LiPO or NiMH
                                    //to use correct battery values.
  #if motor_battery_chem == LIPO
    double m_V1=LiPO_V1;
    double m_V0=LiPO_V0;
    double m_Vsafe=LiPO_Vsafe;
  #elif motor_battery_chem == NiMH
    double m_V1=NiMH_V1;
    double m_V0=NiMH_V0;
    double m_Vsafe=NiMH_Vsafe;
  #endif
  // Motor Battery
    double  cells_per_motor_battery = motor_battery_cells;   // 2S or 3S
    double  battery_motor_V1 = cells_per_motor_battery * m_V1;       // volts, 90% point on LiPO curve at 1C
    double  battery_motor_V0 = cells_per_motor_battery * m_V0;       // volts, 10% point on LiPO curve at 1C
    double  battery_motor_dV  = battery_motor_V1 - battery_motor_V0;
    double  battery_motor_Vsafe = cells_per_motor_battery * m_Vsafe; // volts, 2.7 volts/cell = discharged voltage
#endif


 /******* Robot Communications Initialization Section*********
 *Enabled when comm is defined as adb in Robot Configuration *
 *************************************************************/
#if comm == adb              // To be included only if adb communication is desired.
  // Adb connection.
  Connection * connection;   // the connection variable holds a pointer (i.e. an address) to a variable of data type Connection
#endif

boolean collisionDetection = FALSE; //

void setup(){
  Serial.begin(57600);
  timer = millis();   // telemetry  
  init_servos();      // Pathfinder and Rosco use servos
  #if comm == adb
      //Serial.print("\r\nStart\n"); <- Use line for troubleshooting in adb mode
      ADB::init();        // potential source of OSCOKIRQ error (see notes)
      // Open an ADB stream to the phone's shell. Auto-reconnect
      connection = ADB::addConnection("tcp:4567", true, adbEventHandler); 
  #endif
 
  // insure motors are off and turn off collisionDetection
  stopMotors();
}

void loop(){
  
  sendData(); // Call to send Data for Control Panel Feedback
  
  #if comm == adb
  // Poll the ADB subsystem.
      ADB::poll();   
  #elif comm==otg  
  
     /******************** Command Acquisitions ******************
     *When using OTG this is where the Command ID and associated *
     *                      bytes are obtained                   *
     *************************************************************/
    if( Serial.available() ) {              // replace with serialEvent() handler
      byte data[5];
      int i = 0;
      
      // we want to leave the loop if the command
      // is CAMERA_HOME because the length is 1. Otherwise
      // the command array is of length 5
      boolean cameraHome = false;
      
      if( Serial.peek() == CAMERA_HOME ) {
        cameraHome = true;
      }
      while(!cameraHome && Serial.available() < 5) {}
      
      while(Serial.available()) {
        data[i] = Serial.read();
        i++;
        if( cameraHome ) break;
      }
      commandHandler(data); // Go To function where you add robots code based on Command Received.
    }
    
  #endif
  
  /***********************************************/
  /***** If using a Pinger detect a collision*****/
  /***********************************************/
  #if pinger == TRUE
      if (collisionDetection){
        if (checkSonar()) {
          // insure motors are off and turn off collisionDetection
          stopMotors();
        collisionDetection = FALSE;   // rover is not moving
          // **** send Emergency Stop message ****
        }
      }
  #endif
  
}  // end of loop

 /******Interrupt Service Routine(s) Command Acquisitions ****
 *When using ADB this is where the Command ID and associated *
 *                     bytes are obtained                    *
 *************************************************************/
#if comm == adb 
  // Event handler for the shell connection. 
  void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)    // declared void in version 1.00
  {
    // Serial.print("adbEventHandler");
    // Data packets contain two bytes, one for each servo, in the range of [0..180]
    if (event == ADB_CONNECTION_RECEIVE){
      commandHandler(data); //Go To function where you add robots code based on Command Received.
    }
  }
#endif 

 /**********************Robot Commands ***********************
 * Robot code is added here. Based on Command ID in data[0]  *
 *      and associated bytes, command received is known      *
 *                     Phone to Arduino                      *
 *************************************************************/ 
void commandHandler(uint8_t * data)
{ 
      Serial.print("rover received command ");
      uint8_t cmd = data[0];
      Serial.print(", ");
      Serial.print(cmd, HEX);
      Serial.print(", ");
      Serial.print(data[1], HEX);
      Serial.print(", ");
      Serial.print(data[2], HEX);
      Serial.print(", ");
      Serial.print(data[3], HEX);
      Serial.print(", ");
      Serial.println(data[4], HEX);
      // assumes only one command per packet
      
      if (cmd == MOVE) {
        /***********************************
        * motion command
        * motordata[1]    left run    (FORWARD, BACKWARD, BRAKE, RELEASE)
        * motordata[2]    left speed  0 - 255
        * motordata[3];   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
        * motordata[4];   right speed 0 - 255
        ***********************************/
        move(data);
        //collisionDetection = (data[1] == 1) || (data[2] == 1);  // set to true if any motor is moving forward
      }
      else if (cmd == CAMERA_MOVE){
        /***********************************
         * pan and tilt command 
         * data[1]    0
         * data[2]    pan degrees (0 to 180)
         * data[3]    0
         * data[4]    tilt degrees (0 to 180)
         ***********************************/
        move_camera(data[2],data[4]);
        //Serial.print("Pan Servo To Position: ");
        //Serial.print(data[2]);
        //Serial.print(", ")
        //Serial.println(data[4]);
      }
          else if (cmd == CAMERA_RESET){
      /***********************************
       * camera home command 
       * pan position = 90 (default), tilt position = 90 (default)
       ***********************************/ 
      reset_camera();
      // Serial.println("Camera Home");
   }
      else if (cmd == CAMERA_HOME){
        /***********************************
         * camera home command 
         * pan position = 90 (default), tilt position = 90 (default)
         ***********************************/ 
        home_camera();
        // Serial.println("Camera Home");
     }
        else if (cmd == READ_EEPROM){
      /***********************************
       * EEPROM
       * Telemetry Table = Robot Capabilities Worksheet
       * data[1]    Address High
       * data[2]    Address Low
       * data[3]    Number Bytes
       ***********************************/
      eeprom_address = word(data[1],data[2]); 
      for (int i=1; i<=data[3]; i++){
        eeprom_data = EEPROM.read(eeprom_address);
        // send_eeprom_data(eeprom_data);
      }
      // Serial.println("Read EEPROM");
   }
   else if (cmd == WRITE_EEPROM){
       /***********************************
       * EEPROM 
       * data[1]    Address High
       * data[2]    Address Low
       * data[3]    Number Bytes
       * data[4]    Data (Most Significant Byte)
       * data[5]    ...
       * data[N]    Data (Least Significant Byte) N 
       ***********************************/
      eeprom_address = word(data[1],data[2]); 
      for (int i=1; i<=data[3]; i++){
        EEPROM.write(eeprom_address, eeprom_data);
      }
      // Serial.println("Write EEPROM");
   }
   else if (cmd == SAFE_ROVER){
      /***********************************
       * safe rover
       * motors to idle
       ***********************************/ 
      safeRover();
   }
}

 /************************  Send Data ************************
 *   This is where sensor value is checked and if different  *
 *  a data packet of three bytes (Command ID and associated) *
 *                is sent from Arduino to Phone              *
 *************************************************************/
  
void sendData(){
     uint16_t data;
     int16_t battery_value;
     int16_t sensor_value;
     static uint16_t pingRangeL=0;
     static uint16_t pingRangeR=0;
     const  int16_t tolerance = 4;    
    
    sensor_value = getPanPosition();
    if (sensor_value != positionPan){
      positionPan = sensor_value;          // update
      sendPacket(0x08, sensor_value);
    }
    sensor_value = getTiltPosition();
    if (sensor_value != positionTilt){
      positionTilt = sensor_value;          // update
      sendPacket(0x09, sensor_value);
    }

#if pinger == TRUE
      data= getRangeLeft();
   if(abs(data-pingRangeL) > tolerance){
      pingRangeL=data;     
      sendPacket(RANGE_LEFT_ID, pingRangeL); // distance sensor data package
      }
      
      data= getRangeRight();
   if(abs(data-pingRangeR) > tolerance){
      pingRangeR=data;     
      sendPacket(RANGE_RIGHT_ID, pingRangeR); // distance sensor data package
      }  
#endif


#if Motor_Battery == TRUE     
        battery_value = readFuelGauge(DIRTY_BAT_PIN,battery_motor_V0,battery_motor_dV,battery_motor_Vsafe);   
    if ( !((batteryMotorsV - tolerance <= battery_value) && (battery_value <= batteryMotorsV + tolerance)) ) {
      if (batteryMotorsV == FLAG){
        if (battery_value > 10){
          // come out of safe mode when battery is charged to at least 10%
          batteryMotorsV = battery_value;
        }
        // stay in safe mode
      } 
      else{
        batteryMotorsV = battery_value;   // update
        if (battery_value == FLAG) {
          // battery has just been depleted
          safeRover();   // Need to actually sleep the rover ****
          // Serial.println("Motor Battery Depleted");    // ****
        }
        else {
          // normal operation
          batteryMotorsV = battery_value;                        // update  
          sendPacket(DIRTY_BATTERY_ID,(uint16_t) battery_value);   // ****
          // Serial.print("Motor Battery Voltage = ");
          // Serial.println(batteryMotorsV);
        }
      }
    }
#endif    
#if Digital_Battery == TRUE     
  battery_value = readFuelGauge(CLEAN_BAT_PIN,battery_digital_V0,battery_digital_dV,battery_digital_Vsafe);
    if ( !((batteryDigitalV - tolerance <= battery_value) && (battery_value <= batteryDigitalV + tolerance)) ) {
      if (batteryDigitalV == FLAG){
        if (battery_value > 10){
          // come out of safe mode when battery is charged to at least 10%
          batteryDigitalV = battery_value;
        }
        // stay in safe mode
      } 
      else{
        batteryDigitalV = battery_value;   // update
        if (battery_value == FLAG) {
          // battery has just been depleted
          safeRover();   // Need to actually sleep the rover      ****
          // Serial.println("Clean Battery Depleted");         // ****
        }
        else {
          // normal operation
          batteryDigitalV = battery_value;            // update  
          sendPacket(CLEAN_BATTERY_ID,(uint16_t) battery_value); // ****
          // Serial.print("Clean Battery Voltage = ");
          // Serial.println(batteryDigitalV);
        }
      }
    }
#endif

 /*   
    if(millis() - timer > 500) {  
   //Add all different parameter that will be worked on
    sensor_value = analogRead(MOTOR1CURRENT_PIN);
    if (sensor_value != currentM1){
      currentM1 = sensor_value;               // update
     sendPacket(MOTOR1_CURRENT_ID,sensor_value);
    }
      
    sensor_value = analogRead(MOTOR2CURRENT_PIN);
    if (sensor_value != currentM2){
      currentM2 = sensor_value;               // update
      sendPacket(MOTOR2_CURRENT_ID,sensor_value);
    }
    sensor_value = analogRead(TEMPERATURE_PIN);
    if (sensor_value != tempChassis); {
      tempChassis = sensor_value;             // update
      sendPacket(TEMP_SENSOR_ID,sensor_value);
    }
    timer = millis(); //reset the timer 
   }
*/   

  
}

void sendPacket(uint8_t id, uint16_t value){
  uint8_t sendData[3];
  
  sendData[0] = id;
  sendData[1] = highByte(value);   //  >> 8
  sendData[2] = lowByte(value);    // & 0xff
  
  #if comm == adb
      connection->write(3, sendData);
  #elif comm == otg
      Serial.write(sendData,3 );
  #endif
}


/***************************************************
 * Important Notes                                 *
 ***************************************************/

// Mapping pins 
// ATmega2560-Arduino http://arduino.cc/en/Hacking/PinMapping2560
// ATmega328-Arduino  http://arduino.cc/en/Hacking/PinMapping168

/* The Adb library has the following modifications made to bring
 * it into compliance Arduino version 1.04 (Compiles without errors)
 *  1. Adb.h, usb.cpp, and max3421e.cpp
 *  2. Change the line: #include "wiring.h" to #include "Arduino.h"
 * Runing on ADK generates a "OSCOKIRQ failed to assert" error.
 *  http://forum.arduino.cc/index.php?topic=68205.0
 *
 * If using the ADB Library found in the arxterra Github then 
 * this has been taken care of already.
 */
 
 /***********************************
 * Reminder: Serial Monitor Baud Rate set to 57600
 * to do list
 * pingers code to stop rover included but not tested/debugged
 * code for pan/tilt offset angle
 * continue to clean up code
 *
 * Servo Note:
 * the min pulse width (default is 544 microseconds) corresponds to the minimum angle (0-degree), shaft rotates fully left. 
 * the max pulse width, (defaults to 2400 microseconds) corresponds to the maximum angle (180-degree), shaft rotates fully right.  
 * rotation is left-to-right clockwise
 ***********************************/
 
 /* future
 * #define ATMEGA_TEMP    8
 * ATmega internal temperature sensor ... Need to research how to read
 *   http://forum.arduino.cc/index.php/topic,38043.0.html
 *   http://forum.arduino.cc/index.php/topic,26299.0.html
 *   Problem is time required when you switch between voltage reference sources
 *   for ADC to stabilize. This would limit how often we check this internal sensor.
 * cell phone temperature...     ArxRover needs to add
 */



 /**************************************************
 * Below the Line                                  *
 ***************************************************/

/* Saved in UserState File on Android Phone
 * public var cameraAdjustForMotion:Boolean = false;
 * public var cameraCanPan:Boolean = true;
 * public var cameraCanTilt:Boolean = true;
 * public var cameraConfigDefault:CameraConfig;
 * public var cameraConfigMotion:CameraConfig;
 * public var cameraIndex:int;
 * public var roverName:String;
 */
 
/* Defaults set in the Control Panel's Control Options pop-up Window 
 * Duty Cycle Steps      6 (4 -12)
 * Polling msec          500 (300 - 800)
 * Minimum Duty Cycle    100 (10 - 140)   *** ArxRover *** 
 * Top Duty Cycle        212 (180 - 255)
 * Motion State Display  ON  (Motion Text Display)
 * Control Tips Display  ON  (Tool Tips)  
 * Range Sensor Display  ON  (Numeric and Graphical Icon of Ultrasonic Data)
 */
