#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <iostream>

#include "hardware/uart.h"

#include "tusb.h"

#include <Adafruit_MPU6050.h>
#include <Wire.h>


#include "Adafruit_BMP3XX.h"

#include <pico/stdio.h>
#include <hardware/uart.h>
#include "tusb.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS


#define SEALEVELPRESSURE_HPA (1013.25)


#include "dataStruct.hpp"

static TransmitData transmitData;


static void core1_entry()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
}

Adafruit_MPU6050 mpu;
void mpu6050_setup();
void mpu6050_loop();


Adafruit_BMP3XX bmp;
void bmp3xx_setup();
void bmp3xx_loop();

SFE_UBLOX_GNSS myGNSS;
void gnss_setup();
void gnss_loop();



void transmit();

int main()
{
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    Wire.setClock(400000);

    Serial2.begin(115200);

    pinMode(D17, OUTPUT);
    digitalWrite(D17, LOW);

    pinMode(D18, OUTPUT);
    digitalWrite(D18, LOW);

    uint32_t t0, t1;

#if 0 //wait for usb
    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1 - t0);
#endif

    mpu6050_setup();
    bmp3xx_setup();
    gnss_setup();
    while(1){
        t0 = time_us_32();
        
        {
            mpu6050_loop();
            bmp3xx_loop();
            gnss_loop();
            transmit();
            Serial.println("");
            Serial.println("");
        }

        t1 = time_us_32();
        
        // printf("\nReadTime (%dus)\n", t1 - t0);

        delay(1000);
    }
    return 0;
}


// Basic demo for accelerometer readings from Adafruit MPU6050


void mpu6050_setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void mpu6050_loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acc X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  

  transmitData.acc = {
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  };

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  Serial.println("");
}





void bmp3xx_setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void bmp3xx_loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  transmitData.bmp.pressure = bmp.pressure / 100.0;
  transmitData.bmp.temp = bmp.temperature;

  // Serial.print("Approx. Altitude = ");
  // Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println(" m");

  Serial.println();
}



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


#define mySerial Serial1

void gnss_setup()
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

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  //   myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //   myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
  // myGNSS.setNMEAOutputPort(Serial);
}

void gnss_loop()
{
  bool fixOk = myGNSS.getGnssFixOk();
  Serial.print(F("FixOk: "));
  Serial.print(fixOk);

  transmitData.gps.fix = fixOk;
      
  long latitude = myGNSS.getLatitude();
  Serial.print(F("   Lat: "));
  Serial.print(latitude * 1e-7);

  transmitData.gps.lat = latitude * 1e-7;

  long longitude = myGNSS.getLongitude();
  Serial.print(F(" Long: "));
  Serial.print(longitude * 1e-7);
  // Serial.print(F(" (degrees * 10^-7)"));

  transmitData.gps.lon = longitude * 1e-7;

  byte SIV = myGNSS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  transmitData.gps.siv = SIV;

  Serial.println();
}


void transmit(){

  // Serial2.write("HelloWorld!");
  // Serial.write()
  // return;
  transmitData.fillCheckSum();

  Serial.print("before check:");
  Serial.println(transmitData.checkCheckSum());

  Serial2.write("${");

  uint8_t *ptr = reinterpret_cast<uint8_t*>(&transmitData);
  for(size_t i = 0; i < sizeof(TransmitData); i++){
    if(ptr[i] != '$'){
      Serial2.write(ptr[i]);
    } else {
      Serial2.write(ptr[i]);
      Serial2.write(ptr[i]);
    }
  }

  Serial2.write("$}");
}