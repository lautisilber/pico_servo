#include "PicoServo.h"

#include "pico/mutex.h"

#define SERVO_DEFAULT_MIN_PULSE_WIDTH_US 1000 // uncalibrated default, the shortest duty cycle sent to a servo
#define SERVO_DEFAULT_MAX_PULSE_WIDTH_US 2000 // uncalibrated default, the longest duty cycle sent to a servo
#define SERVO_DEFAULT_MIN_ANGLE 0             // uncalibrated default, the longest duty cycle sent to a servo
#define SERVO_DEFAULT_MAX_ANGLE 180           // uncalibrated default, the longest duty cycle sent to a servo
#define SERVO_PERIOD_US 20000
#define MAX_SERVOS 8 // number of PIO state machines available, assuming nobody else is using them

#define clamp(v, left, right) ((v) > (right) ? (right) : ((v) < (left) ? (left) : (v)))
#define map(v, in_min, in_max, out_min, out_max) \
    (((v) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#define MUTEX_TIMEOUT_MS 3000

#ifndef PICO_PIO_PWM_NO_MUTEX
#define MUTEX_BLOCK(mux, ...)       \
    do                              \
    {                               \
        mutex_enter_blocking(&mux); \
        __VA_ARGS__                 \
        mutex_exit(&mux);           \
    } while (0)
#define MUTEX_BLOCK_TIMEOUT(mux, timeout_ms, ...)      \
    do                                                 \
    {                                                  \
        if (!mutex_enter_timeout_ms(&mux, timeout_ms)) \
            return false;                              \
        __VA_ARGS__                                    \
        mutex_exit(&mux);                              \
    } while (0)
#else
#define MUTEX_BLOCK(mux, ...) \
    do                        \
    {                         \
        __VA_ARGS__           \
    } while (0)
#define MUTEX_BLOCK_TIMEOUT(mux, timeout_ms, ...) \
    do                                            \
    {                                             \
        __VA_ARGS__                               \
    } while (0)
#endif

bool pico_servo_init(struct PicoServo *servo, uint8_t pin, bool inverted)
{
    bool res = pico_pio_pwm_init(&servo->pwm, pin, inverted);
    if (!res)
        return false;

    // blocking should be ok since we just created it
    MUTEX_BLOCK(servo->pwm.mux,
    
        servo->min_us = SERVO_DEFAULT_MIN_PULSE_WIDTH_US;
        servo->max_us = SERVO_DEFAULT_MAX_PULSE_WIDTH_US;
    
        servo->min_angle = SERVO_DEFAULT_MIN_ANGLE;
        servo->max_angle = SERVO_DEFAULT_MAX_ANGLE;

    );

    return true;
}

bool pico_servo_set_min_max_us(struct PicoServo *servo, uint32_t min_us, uint32_t max_us)
{
    MUTEX_BLOCK_TIMEOUT(servo->pwm.mux, MUTEX_TIMEOUT_MS,
    
        servo->min_us = min_us;
        servo->max_us = max_us;

    );
    return true;
}

bool pico_servo_set_min_max_angle(struct PicoServo *servo, uint8_t min_angle, uint8_t max_angle)
{
    MUTEX_BLOCK_TIMEOUT(servo->pwm.mux, MUTEX_TIMEOUT_MS,

        servo->min_angle = min_angle;
        servo->max_angle = max_angle;

    );
    return true;
}

void pico_servo_attach(struct PicoServo *servo)
{
    MUTEX_BLOCK(servo->pwm.mux,

        pico_pio_pwm_set_period_us(&servo->pwm, SERVO_PERIOD_US);

    );
}

void pico_servo_release(struct PicoServo *servo)
{
    MUTEX_BLOCK(servo->pwm.mux,

        pico_pio_pwm_release(&servo->pwm);

    );
}

void pico_servo_deinit(struct PicoServo *servo)
{
    MUTEX_BLOCK(servo->pwm.mux,
        
        pico_pio_pwm_deinit(&servo->pwm);

    );
}

static inline uint32_t pico_servo_angle_to_us(struct PicoServo *servo, uint8_t angle)
{
    return map((uint32_t)angle, (uint32_t)servo->min_angle, (uint32_t)servo->max_angle,
               servo->min_us, servo->max_us);

    // mapping
    // uint32_t dx = (uint32_t)servo->min_angle - (uint32_t)servo->max_angle;
    // uint32_t in_span = (uint32_t)servo->max_angle - (uint32_t)servo->min_angle;
    // uint32_t out_span = servo->max_us - servo->min_us;
    // return (dx * out_span) / in_span + servo->min_us;
}

bool pico_servo_set_angle(struct PicoServo *servo, uint8_t angle)
{
    bool res;
    MUTEX_BLOCK(servo->pwm.mux,
        
        res = servo->pwm.claimed;

        if (res)
        {
            angle = clamp(angle, servo->min_angle, servo->max_angle);
            const uint32_t duty_us = pico_servo_angle_to_us(servo, angle);
            res = pico_pio_pwm_set_duty_us(&servo->pwm, duty_us);
            if (res)
                servo->angle = angle;
        }

    );
    return res;
}

bool pico_servo_sweep(struct PicoServo *servo, uint8_t goal_angle, uint32_t delay_ms, uint32_t resolution_us)
{
    bool res;
    MUTEX_BLOCK(servo->pwm.mux,
        
        res = servo->pwm.claimed;

        if (res)
        {
            goal_angle = clamp(goal_angle, servo->min_angle, servo->max_angle);

            // res is true
            if (servo->angle != goal_angle)
            {
                int32_t curr_us = pico_servo_angle_to_us(servo, servo->angle);
                int32_t final_us = pico_servo_angle_to_us(servo, goal_angle);
                const int8_t dir = (goal_angle > servo->angle ? 1 : -1);
                const int32_t step = (int32_t)dir * resolution_us;

                for (int32_t us = curr_us; (dir > 0 && us <= final_us) || (dir < 0 && us >= final_us); us += step)
                {
                    // // once it's set to false, it stays false
                    // res &= pico_pio_pwm_set_duty_us(&servo->pwm, us);
                    if (!pico_pio_pwm_set_duty_us(&servo->pwm, us))
                    {
                        res = false;
                        break;
                    }
                    sleep_ms(delay_ms);
                }

                if (res)
                    servo->angle = goal_angle;
            }
        }

    );

    return res;
}