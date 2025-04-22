#include "PicoPioPWM.h"

#define max(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

// Write `period` to the input shift register
static inline bool pico_pio_pwm_set_period(struct PicoPioPWM *pwm, uint32_t period)
{
    if (!pwm->claimed) return false;
    pio_sm_set_enabled(pwm->pio, pwm->sm, false);
    pio_sm_put_blocking(pwm->pio, pwm->sm, period);
    pio_sm_exec(pwm->pio, pwm->sm, pio_encode_pull(false, false));
    pio_sm_exec(pwm->pio, pwm->sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pwm->pio, pwm->sm, true);
    pwm->period = period;
    return true;
}

// Write `level` to TX FIFO. State machine will copy this into X.
static inline bool pico_pio_pwm_set_level(struct PicoPioPWM *pwm, uint32_t level)
{
    if (!pwm->claimed) return false;
    pio_sm_put_blocking(pwm->pio, pwm->sm, (pwm->inverted ? pwm->period - level : level));
    pwm->level = level;
    return true;
}

bool pico_pio_pwm_init(struct PicoPioPWM *pwm, uint8_t pin, bool inverted)
{
    pwm->inverted = inverted;
    if (pwm->claimed)
    {
        if (pwm->pin == pin)
            return true;
    }

    // pwm->pio = pio0;
    // pwm->sm = 0;
    // int offset_res = pio_add_program(pwm->pio, &pwm_program);
    // if (offset_res < 0)
    //     return false;
    // pwm->offset = (uint)offset_res;

    pwm->pin = pin;

    bool res = pio_claim_free_sm_and_add_program_for_gpio_range(
        &pwm_program, &pwm->pio, &pwm->sm, &pwm->offset, pwm->pin, pwm->pin, true);
    if (!res)
        return false;

    pwm_program_init(pwm->pio, pwm->sm, pwm->offset, pwm->pin);
    pwm->claimed = true;
    return true;
}

bool pico_pio_pwm_release(struct PicoPioPWM *pwm)
{
    if (!pwm->claimed) return false;
    pico_pio_pwm_set_level(pwm, 0);
    return true;
}

void pico_pio_pwm_deinit(struct PicoPioPWM *pwm)
{
    if (!pico_pio_pwm_release(pwm)) return;
    pio_remove_program_and_unclaim_sm(&pwm_program, pwm->pio, pwm->sm, pwm->offset);
    pwm->claimed = false;
}

static inline uint32_t us_to_pio_instructions(uint32_t us)
{
    /*
     * the pio runs one instruction per clock cycle
     *
     * now we can calculate the number of pio instructions
     * that occur per microsecond (uHz)
     *
     * then we can take that and calculate how many pio
     * instructions must occur before it's been <period_us>
     * microseconds
     */
    const uint32_t pio_freq_hz = clock_get_hz(clk_sys);
    const uint32_t pio_freq_uhz = pio_freq_hz / 1000000;
    const uint32_t n_pio_instructions = us * pio_freq_uhz;
    return n_pio_instructions;
}

static inline uint32_t us_to_pwm_pio_program_cycles(uint32_t us)
{
    /*
     * we take the us the pio takes per instruction
     *
     * then we have to take into account that out pio program
     * runs 3 instructions per pwm cycle and that's it!
     */
    const uint32_t n_pio_instructions = us_to_pio_instructions(us);
    const uint32_t pio_instructions_per_pwm_cycle = 3;
    const uint32_t n_pwm_pio_cycles = n_pio_instructions / pio_instructions_per_pwm_cycle;
    return n_pwm_pio_cycles;
}

bool pico_pio_pwm_set_period_us(struct PicoPioPWM *pwm, uint32_t period_us)
{
    if (!pwm->claimed) return false;
    const uint32_t period = us_to_pwm_pio_program_cycles(period_us);
    pico_pio_pwm_set_period(pwm, period);
    return true;
}

bool pico_pio_pwm_set_duty_us(struct PicoPioPWM *pwm, uint32_t duty_us)
{
    if (!pwm->claimed) return false;
    const uint32_t level = us_to_pwm_pio_program_cycles(duty_us);
    pico_pio_pwm_set_level(pwm, min(level, pwm->period));
    return true;
}