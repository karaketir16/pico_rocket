#include <Arduino.h>

#include <stdint.h>
#include <pico/stdio.h>
#include <hardware/uart.h>
#include "tusb.h"


int main(){
    stdio_init_all(); 

    Serial1.begin(115200);  
    Serial.begin(115200);  

    uint32_t t0, t1;

    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1 - t0);

    
    while(1){
        while(Serial1.available()){
          Serial.write(Serial1.read());
        }

        while(Serial.available()){
          Serial1.write(Serial.read());
        }
    }
}