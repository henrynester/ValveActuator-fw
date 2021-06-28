#pragma once
#include "i2c_comms.h"

class Solenoids
{
public:
    void init();
    void run(SolenoidValvesControl_t *control);
};