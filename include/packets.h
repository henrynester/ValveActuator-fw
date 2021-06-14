#include <stdint.h>
#pragma once

enum Command_t : uint8_t
{
    SET_ACTUATOR_DATA = 0x01,
    REQUEST_SENSOR_DATA = 0x02,
};

struct ActuatorData_t
{
    uint8_t goal_pos;
};

struct SensorData_t
{
    uint8_t pos;
    uint8_t goal_pos;
    int8_t speed;
    uint8_t current_dA;
    uint16_t sensor_A1, //0-1023, unscaled value
        sensor_A2,
        sensor_A3;
    struct Homing_t
    {
        bool limit_switch : 1;
        bool has_homed : 1;
    } homing;
    struct Faults_t
    {
        bool i2c_fault : 1;
        bool drv_failsafe : 1;
        bool open_circuit : 1;
        bool motor_stall : 1;
        bool motor_slip_encoder_drift : 1;
        bool homing_timeout : 1;
        bool limit_switch_stuck : 1;
    } faults;
};