#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "tusb.h"
#include "LoRa_E32.h"


int main() {
    stdio_init_all();

    uint32_t t0,t1;

    t0 = time_us_32();
    while (!tud_cdc_connected()) {}
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1-t0);

    LoRa_E32 lora(&Serial1, 19, 17, 18, UART_BPS_RATE_9600);

    bool l = lora.begin();
    printf("Lora start: %d\n\r", l);

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



    return 0;
}