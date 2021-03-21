/* INA228
Andreas Jonsson

*/

#include <Arduino.h>
#include <Wire.h>
#include "INA228.h"

#define SHUNT_RES (40e-3)         // Rsense in ohm
#define INA228_ADDRESS 0x40       // INA228 i2c address


INA228 ina1(INA228_ADDRESS); // initialize INA1 for i2c

void  setup()
{
  Wire.begin();             // initaiate Arduino i2c lib
  Serial.begin(9600);       // Start Serial monitor at 9600baud
  delay(1000);
  Serial.println();
  Serial.println(F("RESTARTED"));
  Serial.println(F("Configure INA228"));
  
  //configure CONFIG_1 Register see INA228 datasheet 
  /*
  ina1.setADCRANGE(1);
  ina1.setCONVDLY(22);
  ina1.setRST(1);

  //configure ADCCONFIG Register
  ina1.setAVG(2);
  ina1.setVTCT(4);
  ina1.setVSHCT(4);
  ina1.setVBUSCT(4);
  ina1.setMODE(0xf);

  ina1.InitINA(SHUNT_RES);

  */
  
  ina1.InitINA(SHUNT_RES);    //set up INA228 register with selected shunt resistor

  Serial.print(F("MANUFACTURER_ID: "));
  Serial.println(ina1.getMANFID(),HEX);
  delay(200);

  Serial.print(F("REV_ID: "));
  Serial.println(ina1.getREV_ID(), HEX);
  delay(200);

  Serial.print(F("DIE_ID: "));
  Serial.println(ina1.getDIEID(), HEX);
  delay(200);

  Serial.println(F("setBOVL_f(4.00V)"));    // set Bus Overvoltage Threshold
  ina1.setBOVL_f(4.50);                     // set threshold in V (4.50V)
  delay(200);

  Serial.println(F("setBUVL_f(2.50V)"));    // set Bus Undervoltage Threshold
  ina1.setBUVL_f(2.50);                     // set threshold in V (2.50V)
  delay(200);

  Serial.println(F("setSOVL(10mA)"));       // set Shunt Overvoltage Threshold
  ina1.setSOVL_f(10e-3);                    // set threshold in A (10mA)
  delay(200);

  Serial.println(F("setSUVL(5mA)"));        // set Shunt Undervoltage Threshold
  ina1.setSUVL_f(5e-3);                     // set threshold in A (5mA)
  delay(200); 
  
}

void loop()
{
  
  delay(2000);
  Serial.print(F("getVBUS_f:"));
  Serial.println(ina1.getVBUS_f(), 2);      // get and print  Busvoltage in V

  Serial.print(F("getDIETEMP_f:"));
  Serial.println(ina1.getDIETEMP_f(), 2);   // get and print  INA228 dietemp in C

  Serial.print(F("getVSHUNT_f:"));
  Serial.println(ina1.getVSHUNT_f(), 4);    // get and print  INA228 Rsense Voltage in V

  float Current = ina1.getCURRENT_f();
  Serial.print(F("calculatedCURRENT:"));    // get and print  INA228 Rsense Current in A
  Serial.println(Current, 8);

  float Power = ina1.getPOWER();
  Serial.print(F("calculatedPower:"));      // get and print  INA228 Rsense Power in W
  Serial.println(Power, 8);

  float Energy = ina1.getENERGY();
  Serial.print(F("calculatedEnergy:"));     // get and print  INA228 Rsense Energy in J
  Serial.println(Energy, 8);

// Check Over and under warnings

  
  if (ina1.getBUSUL())
    Serial.println(F("Bus Under Voltage triggerd"));

  if (ina1.getBUSOL())
    Serial.println(F("Bus Over Voltage triggerd"));

  if (ina1.getSHNTUL())
    Serial.println(F("Shunt Under Voltage triggerd"));

  if (ina1.getSHNTOL())
    Serial.println(F("Shunt Over Voltage triggerd"));

  
  Serial.println();
}