#include "Solenoids.h"
#include "Arduino.h"
#include "config.h"

void Solenoids::init()
{
    pinMode(ENGINE_VENT_SOL, OUTPUT);
    pinMode(FUEL_MAIN_SOL, OUTPUT);
}

void Solenoids::run(SolenoidValvesControl_t *control)
{
    digitalWrite(ENGINE_VENT_SOL, control->engine_vent_valve_close);
    digitalWrite(FUEL_MAIN_SOL, control->main_fuel_valve_open);
}