/*
    PIO-based Servo class for Rasperry Pi Pico RP2040

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
    A servo is activated by creating an instance of the Servo class passing
    the desired pin to the attach() method.
    The servos are pulsed in the background using the value most recently
    written using the write() method.

    The methods are:
     Servo - Class for manipulating servo motors connected to Arduino pins.
       attach(pin)  - Attaches a servo motor to an i/o pin.
       attach(pin, min, max) - Attaches to a pin setting min and max values in microseconds
                               default min is 1000, max is 2000
       write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
       writeMicroseconds() - Sets the servo pulse width in microseconds
       read()      - Gets the last written servo pulse width as an angle between 0 and 180.
       readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
       attached()  - Returns true if there is a servo attached.
       detach()    - Stops an attached servos from pulsing its i/o pin.
*/

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
extern void pico_servo_deinit(struct PicoServo *servo);

extern bool pico_servo_set_angle(struct PicoServo *servo, uint8_t angle);

// should be initialized externally
extern bool pico_servo_sweep(struct PicoServo *servo, uint8_t goal_angle, uint32_t delay_ms, uint32_t resolution_us);

#if __cplusplus
}
#endif

#endif /* _PICO_SERVO_H_ */