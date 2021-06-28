#include "Encoder.h"
#include <Arduino.h>
#include "config.h"

// void encoderInterrupt()
// {
//     g_enc.update();
// }

void Encoder::init()
{
    pinMode(encA_pin, INPUT_PULLUP);
    pinMode(encB_pin, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(ENCA), encoderInterrupt, CHANGE);
}

void Encoder::increment()
{
    if (count < UINT16_MAX)
        count++;
}
void Encoder::decrement()
{
    if (count > 0)
        count--;
}

void Encoder::update()
{
    if (digitalRead(encA_pin))
    {
        //F0 (ENCA) rising interrupt. check ENCB to see whether rotation is fwd or rev
        if (digitalRead(encB_pin))
            decrement();
        else
            increment();
    }
    else
    {
        //falling interrupt
        if (digitalRead(encB_pin))
            increment();
        else
            decrement();
    }
}

uint16_t Encoder::getRawCount()
{
    //thread-safe?
    //count could be modified if a tick occurs during this function's execution
    //assume that the return is an atomic operation...
    return count;
}
void Encoder::zero()
{
    count = 0;
}