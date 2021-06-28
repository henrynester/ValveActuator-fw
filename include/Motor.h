#pragma once //only include header once on compilation
#include <stdint.h>

// enum DriveMode
// {
//     FORWARD,
//     REVERSE,
//     BRAKE, //outputs both high; motor acts as generator and locks in place
//     FREE   //outputs high-impedance; motor spins "freely" (it has a gearbox...)
// };

class Motor
{
public:
    Motor(uint8_t pwm_pin, uint8_t dir_pin)
        : pwm_pin(pwm_pin),
          dir_pin(dir_pin){};
    void init();
    //-100 (rev) ... 0 (brake) ... 100 (fwd)
    void setSpeed(int8_t speed);

private:
    uint8_t pwm_pin, dir_pin;
};