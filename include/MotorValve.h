#pragma once
#include "Motor.h"
#include "Encoder.h"
#include "i2c_comms.h"

class MotorValve
{
private:
    Motor motor;
    Encoder encoder;
    uint8_t limit_pin;
    uint16_t encoder_max;

    uint16_t t_move_start;
    uint8_t stopped_pos;

public:
    MotorValve(uint8_t pwm_pin, uint8_t dir_pin, uint8_t encA_pin, uint8_t encB_pin, uint8_t limit_pin, uint16_t encoder_max)
        : motor(pwm_pin, dir_pin),
          encoder(encA_pin, encB_pin),
          limit_pin(limit_pin),
          encoder_max(encoder_max){};
    void init();
    void run(MotorValveStatus_t *status);
};