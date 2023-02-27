#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "LoRa_E32.h"

#include <iostream>

#include "hardware/uart.h"

#include "tusb.h"

#define UART_ID uart0
#define BAUD_RATE 115200

void core1_entry() {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}


int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    uint32_t t0,t1;

    t0 = time_us_32();
    while (!tud_cdc_connected()) {}
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1-t0);

    LoRa_E32 lora(&Serial1, 19, 17, 18, UART_BPS_RATE_9600);

    bool l = lora.begin();
    printf("%d\n\r", l);

    // lora.cleanUARTBuffer();
    while (Serial1.available())
    {
        Serial1.read();
    }
    


    {
        auto conf = lora.getConfiguration();
        printf("get Conf: %s\n\r", conf.status.getResponseDescription().c_str());
        Configuration *s = (Configuration *)conf.data;

        for(int i = 0 ; i < sizeof(Configuration); i ++){
            printf("0x%02x ", ((uint8_t*)(conf.data))[i]);
        }

        printf("\r\n");
        printf("add L:0x%02x H:0x%02x\r\n", s->ADDL, s->ADDH);

        s->SPED.uartBaudRate = UART_BPS_115200;
        s->SPED.airDataRate = AIR_DATA_RATE_000_03;
        s->SPED.uartParity = MODE_00_8N1;

        s->OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
        s->OPTION.transmissionPower = POWER_20;
        s->OPTION.fec = FEC_0_OFF;

        s->CHAN = 38;

        s->ADDH = BROADCAST_ADDRESS;
        s->ADDL = BROADCAST_ADDRESS;
                

        auto res = lora.setConfiguration(*s, WRITE_CFG_PWR_DWN_SAVE);
        printf("set conf: %s\n\r", res.getResponseDescription().c_str());

    }

    Serial1.end();
    Serial1.begin(115200);
    
    delay(1000);

        // lora.cleanUARTBuffer();
    while (Serial1.available())
    {
        Serial1.read();
    }



    while(1){
        printf("Send Data: \r\n");
        // t0 = time_us_32();
        // auto res = lora.sendMessage(data, 6);
        // t1 = time_us_32();
        // printf("%s\n\r", res.getResponseDescription().c_str());
        // printf("\nSend Data Done: (%dms) %s\r\n", (t1-t0) / 1000);

        Serial1.println("Hello World!");

        sleep_ms(250);
    }

    return 0;
}