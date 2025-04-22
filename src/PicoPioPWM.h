#ifndef _PICO_PIO_PWM_H_
#define _PICO_PIO_PWM_H_

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pwm.pio.h"

#if __cplusplus
extern "C" {
#endif

struct PicoPioPWM
{
    uint8_t pin;

    bool claimed;
    bool inverted;
    uint32_t period;
    uint32_t level;

    PIO pio;
    uint sm;
    uint offset;
};

extern bool pico_pio_pwm_init(struct PicoPioPWM *pwm, uint8_t pin, bool inverted);
extern bool pico_pio_pwm_release(struct PicoPioPWM *pwm);
extern void pico_pio_pwm_deinit(struct PicoPioPWM *pwm);

extern bool pico_pio_pwm_set_period_us(struct PicoPioPWM *pwm, uint32_t period_us);
extern bool pico_pio_pwm_set_duty_us(struct PicoPioPWM *pwm, uint32_t duty_us);

#if __cplusplus
}
#endif

#endif /* _PICO_PIO_PWM_H_ */