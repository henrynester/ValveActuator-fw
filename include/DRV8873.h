#pragma once //only include header once on compilation
#include <stdint.h>

// enum DriveMode
// {
//     FORWARD,
//     REVERSE,
//     BRAKE, //outputs both high; motor acts as generator and locks in place
//     FREE   //outputs high-impedance; motor spins "freely" (it has a gearbox...)
// };

class DRV8873
{
private:
    int8_t speed;

public:
    void init();
    //-100 (rev) ... 0 (brake) ... 100 (fwd)
    void setSpeed(int8_t speed);
    int8_t getSpeed();
    uint16_t readCurrentdA(); //dA = 1/10 Amp
    uint8_t isInFault();
};