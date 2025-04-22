#include <stdio.h>

#include "pico/stdlib.h"
#include <PicoPioPWM.h>

#define PIN 11

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    puts("PicoPioPWM example: sweep");

    const uint32_t period_us = 20 * 1000;
    const uint32_t duty_us_min = 1200;
    const uint32_t duty_us_max = 1700;
    const uint32_t step = 5;

    // It's important to initialize the struct
    // to 0!
    // (mainly <mutex_t mux> inside the struct
    //  should be initialized to 0!)
    struct PicoPioPWM pwm = {};

    for (;;)
    {
        if (!pico_pio_pwm_init(&pwm, PIN, true))
        {
            printf("ERROR %lu\n", __LINE__);
            return 1;
        }
        pico_pio_pwm_set_period_us_safe(&pwm, period_us);
        for (uint32_t i = duty_us_min; i <= duty_us_max; i += step)
        {
            pico_pio_pwm_set_duty_us_safe(&pwm, i);
        }
        pico_pio_pwm_release_safe(&pwm);
        sleep_ms(2000);
        for (uint32_t i = duty_us_max; i >= duty_us_min; i -= step)
        {
            pico_pio_pwm_set_duty_us_safe(&pwm, i);
        }
        pico_pio_pwm_deinit_safe(&pwm);
        sleep_ms(500);
    }
}
