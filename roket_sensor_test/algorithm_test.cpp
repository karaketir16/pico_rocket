#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <iostream>
#include <hardware/uart.h>
#include "tusb.h"
#include "Arduino.h"
#include <utility>

#include <math.h>

double test_rocket_velocity(double t) {
    t /= 3;


    double v = -4.9 * pow(t, 2) + 100 * t;
    v *= 0.6;

    if(v < -150){
        return -150;
    }

    return v;
}

struct Algorithm
{
    enum class State
    {
        Initial,
        Setup,
        Waiting,
        Rising,
        Apogee,
        Falling_1,
        MainDeploy,
        Falling_2,
        Done

    } state = State::Initial;

    double baseHeight = 0;
    double height = 0;

    State run(double __RawHeight, double velocity)
    {
        height = __RawHeight - baseHeight;

        switch (state)
        {
        case Algorithm::State::Initial:
        {
            state = State::Setup;
        }
        break;
        case Algorithm::State::Setup:
        {
            baseHeight = __RawHeight;
            height = 0;
            state = State::Waiting;
        }
        break;
        case Algorithm::State::Waiting:
        {
            if (height > 100)
            {
                state = State::Rising;
            }
        }
        break;
        case Algorithm::State::Rising:
        {
            if (velocity < 5)
            {
                state = State::Apogee;
            }
        }
        break;
        case Algorithm::State::Apogee:
        {
            state = State::Falling_1;
        }
        break;
        case Algorithm::State::Falling_1:
        {
            if(height < 600){
                state = State::MainDeploy;
            }
        }
        break;
        case Algorithm::State::MainDeploy:
        {
            state = State::Falling_2;
        }
        break;
        case Algorithm::State::Falling_2:
            if(abs(velocity) < 1){
                state = State::Done;
            }
            break;
        case Algorithm::State::Done:
            break;

        default:
            break;
        }

        return state;
    }
};

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


// #define ApogeePIN D0
// #define MainDeployPIN D1

#define ApogeePIN       D0
#define MainDeployPIN   LED_BUILTIN

struct PinDriver
{
    static const auto pinA = ApogeePIN;
    static const auto pinB = MainDeployPIN;

    bool aFired = false;
    double aFiredTime = 0;
    bool bFired = false;
    double bFiredTime = 0;

    double time = 0;

    PinDriver(){
        pinMode(pinA, OUTPUT);
        digitalWrite(pinA, LOW);

        pinMode(pinB, OUTPUT);
        digitalWrite(pinB, LOW);
    }

    void drivePins(double t){

        time = t;

        if(aFired && (t - aFiredTime) > 15) {//second
            digitalWrite(pinA, LOW);
            aFired = false;
        }

        if(bFired && (t - bFiredTime) > 15) {//second
            digitalWrite(pinB, LOW);
            bFired = false;
        }

        if(aFired){
            Serial.println("aFired");
        }
        if(bFired){
            Serial.println("bFired");
        }
    }

    void fireA(){
        aFiredTime = time;
        aFired = true;
        digitalWrite(pinA, HIGH);
    } 
    void fireB(){
        bFiredTime = time;
        bFired = true;
        digitalWrite(pinB, HIGH);
    }
};




int main()
{
    stdio_init_all();
    // multicore_launch_core1(core1_entry);

    pinMode(D3, INPUT);

    Serial.begin(115200);

    uint32_t t0, t1;

    t0 = time_us_32();
    while (!tud_cdc_connected())
    {
    }
    t1 = time_us_32();
    Serial.printf("\nusb host detected! (%dus)\n", t1 - t0);

    Algorithm alg;
    PinDriver pins;

    double t = 0;
    
    double last_time = 0;

    double initH = 150;
    double last_height = initH;
    while(1){
        double velocity = test_rocket_velocity(t);
        
        if(velocity < 0 && last_height < 650){
            velocity = -7;
        }

        double height = last_height + velocity * (t - last_time);

        if(height < 150){
            velocity = 0;
        }

        auto res = alg.run(height, velocity);

        pins.drivePins(t);

        if(res == Algorithm::State::Apogee){
            pins.fireA();
        } 

        if(res == Algorithm::State::MainDeploy){
            pins.fireB();
        } 

        Serial.printf("state: %d, time: %lf, h: %lf, v: %lf\r\n", (int) res, t, height, velocity);

        last_height = height;
        last_time = t;

        if(res == Algorithm::State::Done){
            break;
        }

        t += 2;
        delay(200);
    }

    Serial.print("Done");

    while(1){
        asm("nop");
    }

    return 0;
}

