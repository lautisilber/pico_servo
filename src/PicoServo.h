#ifndef _PICO_SERVO_H_
#define _PICO_SERVO_H_

#include "pico/stdlib.h"
#include "PicoPioPWM.h" // Ensure this header defines PicoPioPWM


#if __cplusplus
extern "C" {
#endif


struct PicoServo {
    struct PicoPioPWM pwm;

    uint8_t angle;

    uint32_t min_us;
    uint32_t max_us;
    uint8_t min_angle;
    uint8_t max_angle;
};

extern bool pico_servo_init(struct PicoServo *servo, uint8_t pin, bool inverted);

extern bool pico_servo_set_min_max_us(struct PicoServo *servo, uint32_t min_us, uint32_t max_us);
extern bool pico_servo_set_min_max_angle(struct PicoServo *servo, uint8_t min_angle, uint8_t max_angle);

extern bool pico_servo_attach(struct PicoServo *servo);
extern bool pico_servo_release(struct PicoServo *servo);
extern bool pico_servo_deinit(struct PicoServo *servo);

extern bool pico_servo_set_angle(struct PicoServo *servo, uint8_t angle);

// should be initialized externally
extern bool pico_servo_sweep(struct PicoServo *servo, uint8_t goal_angle, uint32_t delay_ms, uint32_t resolution_us);

#if __cplusplus
}
#endif

#endif /* _PICO_SERVO_H_ */