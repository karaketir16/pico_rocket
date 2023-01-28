#include <TinyGPSPlus.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 115200;

#define ss Serial1

void push_data(const void* data, uint16_t len){
    uint16_t wordsCount = ((len + 1) / 4);

    const uint32_t *words = (const uint32_t *)data;

    for (size_t i = 0; i < wordsCount; i++)
    {
        multicore_fifo_push_blocking(words[i]);
    }
    
}


void pop_data(void* data, uint16_t len){
    uint16_t wordsCount = ((len + 1) / 4);

    uint32_t *words = (uint32_t *)data;

    for (size_t i = 0; i < wordsCount; i++)
    {
        words[i] = multicore_fifo_pop_blocking();
    }
}

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device

void displayInfo();


long lastTime = 0;
void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

  lastTime = millis();
}

void loop()
{
  // This send information every time centisecond changes,  (that cause 100ms delay, not important for this project)
  
  while (ss.available() > 0){
    char c = ss.read();
    gps.encode(c);
  }

  if((millis() - lastTime) >= 100){
    lastTime = millis();
    push_data(&gps, sizeof(gps));
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo(TinyGPSPlus gps__)
{
  Serial.println(F("From core0: ")); 
  Serial.print(F("Location: ")); 
  if (gps__.location.isValid())
  {
    Serial.print(gps__.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps__.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps__.date.isValid())
  {
    Serial.print(gps__.date.month());
    Serial.print(F("/"));
    Serial.print(gps__.date.day());
    Serial.print(F("/"));
    Serial.print(gps__.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps__.time.isValid())
  {
    if (gps__.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps__.time.hour());
    Serial.print(F(":"));
    if (gps__.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps__.time.minute());
    Serial.print(F(":"));
    if (gps__.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps__.time.second());
    Serial.print(F("."));
    if (gps__.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps__.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


#include <pico/stdio.h>
#include <hardware/uart.h>
#include "pico/multicore.h"
#include "tusb.h"

void core1_entry(){
  setup();
  while (1){
    loop();
  }
  
}

int main(){
    stdio_init_all();   

    multicore_launch_core1(core1_entry);

    uint32_t t0, t1;

    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1 - t0);

    
    while(1){
        t0 = time_us_32();
        TinyGPSPlus gps__;
        pop_data(&gps__, sizeof(gps__));
        displayInfo(gps__);
        t1 = time_us_32();
        // printf("\nPop Time passes (%dus)\n", t1 - t0);
    }
}