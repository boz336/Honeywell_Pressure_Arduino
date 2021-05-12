/// WORKING CODE to read HSCDANN1.6BASA5 sensor
////
/// Helpful websites:  
/// https://sensing.honeywell.com/index.php?ci_id=151133
/// https://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf
/// https://github.com/AlexSatrapa/SSC/blob/master/SSC.cpp
/// https://forum.arduino.cc/t/reading-honeywell-hsc-sensors-over-spi/341872
////
#include <SPI.h>
#include <Arduino.h>
#define OUTPUT_MIN 1638        // 1638 counts (10% of 2^14 counts or 0x0666)
#define OUTPUT_MAX 14745       // 14745 counts (90% of 2^14 counts or 0x3999)
#define PRESSURE_MIN 0        // min is 0 for sensors that give absolute values
//#define PRESSURE_MAX 1600   // 1.6bar wantesults in mbar
#define PRESSURE_MAX  23.5136  //PSI for 1.6bar 


uint8_t chipSelectPin1 = 3;


void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(chipSelectPin1, OUTPUT);
  digitalWrite(chipSelectPin1, HIGH);
}

void loop()
{
   float val1 = 1.000 * ( ((float)readSensor(chipSelectPin1) - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN) / (OUTPUT_MAX - OUTPUT_MIN)) + PRESSURE_MIN;
  Serial.print("  Pr_psi =  "); Serial.println(val1,6);
  delay(10);
}

int16_t readSensor (uint8_t selectPin) {

  SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE1)); // Set to 700kHz, MSB and MODE1
  digitalWrite(selectPin, LOW);       //pull Chipselect Pin to Low

  int inByte_1 = SPI.transfer(0x00);  // Read first Byte of Pressure
  int inByte_2 = SPI.transfer(0x00);  // Read second Byte of Pressure
  int inByte_3 = SPI.transfer(0x00);  // Read first Byte of Temperature
  int inByte_4 = SPI.transfer(0x00);  // Read second Byte of Temperature
  int inMByte_1 = inByte_1 | B00111111;


  digitalWrite(selectPin, HIGH);      //pull Chipselect Pin to High
  SPI.endTransaction();               //end SPI Transaction
// Serial.print("Chipselect = "); Serial.print(selectPin, DEC); Serial.print("   ");
//  Serial.print("Byte_1 = "); Serial.print(inByte_1, DEC); Serial.print("   ");
//  Serial.print("Byte_2 = "); Serial.print(inByte_2, DEC); Serial.print("   ");
//  Serial.print("Byte_3 = "); Serial.print(inByte_3, DEC); Serial.print("   ");
//  Serial.print("Byte_4 = "); Serial.print(inByte_4, DEC); Serial.print("   ");
//  Serial.print("inMByte_1 = "); Serial.print(inMByte_1, DEC); Serial.print("   ");


  int16_t pressure_dig = (((inByte_1 & 0x3F)) << 8) | inByte_2 ;
//   pressure_dig = pressure_dig | inByte_2;

 inByte_3 = inByte_3 << 3; //Shift first Temperature byte 3 left
  
  float realTemp = ((float)inByte_3 * 200 / 2047) - 50; //Convert Digital value to °C

  int8_t stat = inByte_1 >> 6;
  Serial.print("STATUS=  "); Serial.print(stat); 
  Serial.print("  Press Count = "); Serial.print(pressure_dig); Serial.print("   ");
  
  Serial.print("  Temp[C]=  "); Serial.print(realTemp); Serial.print("     ");

  return pressure_dig; //return digital Pressure Value
}
