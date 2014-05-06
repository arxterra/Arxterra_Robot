// The function readFuelGauge returns a number between 0 and 100
// if battery within operational limits and a number greater than
// 100 (actually 128) for undervoltage (discharged) condition.

// Optimization
// Translate to look-up table as defined for IR sensor in EE444
// Work in 2 byte DN not 8 byte double

// Voltage Divider Network
// Assuming common LiPO 3S design (highest voltage) for all battery types 
const double R1 = 22000;  // 22K resistor from battery to analog input
const double R2 = 12000;  // 12K resistor from analog input to ground
const double K = (R2/(R1 + R2))*(1024/5.0);  // where DN = K * V (see reference documentation) 
                                             // Vref = 5.0 +/- 0.1v and a 10-bit DAC
                                             // for 3S LiPO  K = 72.28235 and 1/K = 0.0138346 
void initFuelGauge(){
  int value;
  Serial.print("7S NiMH: ");
  Serial.print("battery_motor_V0 = ");
  Serial.print(battery_motor_V0);
  Serial.print(", battery_motor_V1 = ");
  Serial.print(battery_motor_V1);
  Serial.print(", battery_motor_dV = ");
  Serial.print(battery_motor_dV);
  Serial.print(", battery_motor_Vsafe = ");
  Serial.println(battery_motor_Vsafe);
  
  Serial.print("8S NiMH: ");
  Serial.print("battery_digital_V0 = ");
  Serial.print(battery_digital_V0);
  Serial.print(", battery_digital_V1 = ");
  Serial.print(battery_digital_V1);
  Serial.print(", battery_digital_dV = ");
  Serial.print(battery_digital_dV);
  Serial.print(", battery_digital_Vsafe = ");
  Serial.println(battery_digital_Vsafe);

/*
  value = readFuelGauge(DIRTY_BAT_PIN,battery_motor_V0,battery_motor_dV,battery_motor_Vsafe);
  Serial.print("8S NiMH");
  value = readFuelGauge(CLEAN_BAT_PIN,battery_digital_V0,battery_digital_dV,battery_digital_Vsafe);
 */ 
}
                                             
int16_t readFuelGauge(int pin,double V0,double dV, double Vsafe){
  int16_t P = FLAG;                          // undervoltage condition  
  /* conversion algorithm */
  double DN = (double) analogRead(pin);      // read adc and cast as double
  double V = DN / K;                         // convert to battery voltage
  if (V > Vsafe){
    double F = 100*(V - V0)/dV;              // convert to fuel gauge reading as a percentage
    P = (uint16_t) constrain(F,0.0,100.0);   // constrain to an integer between 0 and 100 and cast to byte
  } 
  return P;
}
