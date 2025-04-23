#include "pico/stdlib.h"
#include <stdio.h>

#include "PicoServo.h"

#define PIN 11

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    puts("PicoServo example: sweep");

    // Important to intitialize to 0!
    struct PicoServo servo = {};

    if (!pico_servo_init(&servo, PIN, true))
    {
        puts("Couldn't initialize servo");
        sleep_ms(5000);
        return 1;
    }

    pico_servo_attach(&servo);
    pico_servo_set_angle(&servo, 90);

    const uint32_t delay_ms = 20;
    const uint32_t resolution_us = 5;

    for (;;)
    {
        printf("angle: %li\n", servo.angle);
        pico_servo_sweep(&servo, 180, delay_ms, resolution_us);
        printf("angle: %li\n", servo.angle);
        pico_servo_sweep(&servo, 0, delay_ms, resolution_us);
    }
}