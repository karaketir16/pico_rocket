#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <iostream>

#include "hardware/uart.h"

#include "tusb.h"

#include "MPU6050.h"
#include <Adafruit_BMP3XX.h>

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 0
#define UART_RX_PIN 1

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

int main_2();
int main_scan();

#define OUTPUT_READABLE_ACCELGYRO

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 accelgyro;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high
// MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

static int16_t ax, ay, az;
static int16_t gx, gy, gz;

static MPU6050 mpu;

#define GRAVITY 9.80665

int main()
{
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uint32_t t0, t1;

    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    printf("\nusb host detected! (%dus)\n", t1 - t0);

    // initialize device
    printf("Initializing I2C devices...\r\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\r\n");
    printf("%s\r\n", mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

    while (1)
    {
        sleep_ms(250);
        // read raw accel/gyro measurements from device
        // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        mpu.getAcceleration(&ax, &ay, &az);
        // these methods (and a few others) are also available
        // accelgyro.getAcceleration(&ax, &ay, &az);
        // accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax);
        Serial.print("\t");
        Serial.print(ay);
        Serial.print("\t");
        Serial.print(az);
        // Serial.print("\t");
        // Serial.print(gx);
        // Serial.print("\t");
        // Serial.print(gy);
        // Serial.print("\t");
        // Serial.print(gz);

        Serial.print("\ta/g:\t");
        Serial.print(ax / 2048.0 * GRAVITY);
        Serial.print("\t");
        Serial.print(ay/ 2048.0 * GRAVITY);
        Serial.print("\t");
        Serial.println(az/ 2048.0 * GRAVITY);


#endif
    }

    return 0;
}
