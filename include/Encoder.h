#pragma once
#include <stdint.h> //for datatypes

class Encoder
{
public:
    void init();
    uint32_t getRawCount();
    double getPosition();
    void update();
    void zero();
    void setMax(uint32_t max);
    uint8_t readLimit();

private:
    volatile uint32_t count;
    uint32_t max;
    void increment(), decrement();
};