#include "MotorValve.h"
#include "Arduino.h"

void MotorValve::init()
{
    encoder.init();
    motor.init();
    pinMode(limit_pin, INPUT_PULLUP);
}

void MotorValve::run(MotorValveStatus_t *status)
{
    status->homing.limit_switch = digitalRead(limit_pin);
    status->pos = (uint8_t)((double)encoder.getRawCount() / encoder_max);

    //update warning flags
    status->faults.homing_timeout = (!status->homing.has_homed) && (millis() > (t_move_start + 5000));
    status->faults.limit_switch_stuck = (status->homing.limit_switch) && (status->pos != 0);

    //use encoder to figure out whether motor is actually rotating
    if (status->speed == 0)
    {
        stopped_pos = status->pos;
        t_move_start = millis();
    }
    uint8_t delta_pos = abs(status->pos - stopped_pos);
    //nonzero speed to driver but encoder doesn't rotate
    status->faults.motor_stuck = (status->speed != 0) && delta_pos < 5 && millis() - t_move_start > 500;
    //zero speed to driver (brake/lock) but encoder rotates
    status->faults.motor_slip_encoder_drift = (status->speed == 0) && delta_pos != 0;

    if (status->homing.has_homed)
    {
        //normal operation
        //control motor speed to move toward goal position
        //simple proportional control (for now)
        //TODO: implement PID pos. ctrl.
        motor.setSpeed(10 * (status->goal_pos - status->pos));
    }
    else
    {
        //we need to home the encoder
        //TODO: have valve move away from zero a bit, then close, to ensure full closure at 0-position
        motor.setSpeed(-100); //reverse to zero position
        if (status->homing.limit_switch)
        {
            status->homing.has_homed = true;
        }
    }
}