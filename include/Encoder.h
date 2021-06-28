#pragma once
#include <stdint.h> //for datatypes

class Encoder
{
public:
    Encoder(uint8_t encA_pin, uint8_t encB_pin)
        : encA_pin(encA_pin),
          encB_pin(encB_pin){};
    void init();
    uint16_t getRawCount();
    void update();
    void zero();

private:
    uint8_t encA_pin, encB_pin;
    volatile uint16_t count;
    void increment(), decrement();
};