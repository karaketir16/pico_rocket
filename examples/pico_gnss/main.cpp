/*
  Reading lat and long via UBX binary commands using UART @38400 baud - free from I2C
  By: Nathan Seidle, Adapted from Example3_GetPosition by Thorsten von Eicken
  SparkFun Electronics
  Date: January 28rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the library and U-Blox for serial port use as well as
  switching the module from the default 9600 baud to 38400.

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We 
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Connect the U-Blox serial TX pin to Uno pin 10
  Connect the U-Blox serial RX pin to Uno pin 11
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(10, 11); // RX, TX. Pin 10 on Uno goes to TX pin on GNSS module.

#define mySerial Serial1

long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to u-blox module.



void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 115200 baud.
  //Loop until we're in sync and then ensure it's at 115200 baud.
  do {
    Serial.println("GNSS: trying 115200 baud");
    mySerial.begin(115200);
    mySerial.println("Hello World");
    if (myGNSS.begin(mySerial) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    mySerial.begin(9600);
    if (myGNSS.begin(mySerial) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 115200");
        myGNSS.setSerialRate(115200);
        delay(100);
    } else {
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

//   myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
//   myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
//   myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
    myGNSS.setNMEAOutputPort(Serial);
}

void navPVT(UBX_NAV_PVT_data_t pvt){
    printf("day: %d month: %d year: %d\r\n", pvt.day, pvt.month, pvt.year);
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    // printf("automatic: %d\r\n", myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.automatic);

    bool fixOk = myGNSS.getGnssFixOk();
    Serial.print(F("FixOk: "));
    Serial.print(fixOk);
        
    long latitude = myGNSS.getLatitude();
    Serial.print(F("   Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }
}


#include <pico/stdio.h>
#include <hardware/uart.h>
#include "tusb.h"


int main(){
    stdio_init_all();   

    uint32_t t0, t1;

    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1 - t0);

    setup();
    while(1){
        loop();
    }
}