#pragma once
#include <Arduino.h>

enum I2CCommand_t
{
    CMD_NONE = 0, //used internally in code. do not send/receive to this cmd
    CMD_CONTROL = 1,
    CMD_STATUS = 2,
};

struct SolenoidValvesControl_t
{
    bool engine_vent_valve_close : 1;
    bool main_fuel_valve_open : 1;
};

struct MotorValveStatus_t
{
    uint8_t pos;
    uint8_t goal_pos;
    int8_t speed;
    struct LimitSwitchStatus_t
    {
        bool limit_switch : 1;
        bool has_homed : 1;
    } homing;
    struct MotorValveFaults_t
    {
        bool motor_stuck : 1;
        bool homing_timeout : 1;
        bool limit_switch_stuck : 1;
        bool motor_slip_encoder_drift : 1;
    } faults;
};

//these are the main structs we send on the I2C bus
struct Control_t
{
    uint8_t fuel_press_valve_goal_pos;
    uint8_t main_ox_valve_goal_pos;
    struct SolenoidValvesControl_t solenoids;
};

struct Status_t
{
    struct MotorValveStatus_t fuel_press_valve;
    struct MotorValveStatus_t main_ox_valve;
    struct SolenoidValvesControl_t solenoids;
};

//I2C
void initI2C();
void onI2CReceive(int);
void onI2CRequest(void);

extern bool connected, just_rxd_control;
extern enum I2CCommand_t last_cmd;

extern struct Status_t status;
extern uint8_t status_tx[sizeof(struct Status_t)]; //idk if this one needs to be volatile - the ISR only acceses it. can't hurt

extern struct Control_t control_rx;