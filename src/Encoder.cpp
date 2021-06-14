#include "Encoder.h"
#include <Arduino.h>
#include "config.h"

Encoder g_enc;

void encoderInterrupt()
{
    g_enc.update();
}

void Encoder::init()
{
    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);
    pinMode(LIMIT, INPUT_PULLUP); //switch to ground with internal pullup
    attachInterrupt(digitalPinToInterrupt(ENCA), encoderInterrupt, CHANGE);
}

void Encoder::increment()
{
    if (count < UINT32_MAX)
        count++;
}
void Encoder::decrement()
{
    if (count > 0)
        count--;
}

void Encoder::update()
{
    if (digitalRead(ENCA))
    {
        //F0 (ENCA) rising interrupt. check ENCB to see whether rotation is fwd or rev
        if (digitalRead(ENCB))
            decrement();
        else
            increment();
    }
    else
    {
        //falling interrupt
        if (digitalRead(ENCB))
            increment();
        else
            decrement();
    }
}

uint8_t Encoder::readLimit()
{
    return !digitalRead(LIMIT); //Active LOW
}

uint32_t Encoder::getRawCount()
{
    //thread-safe?
    //count could be modified if a tick occurs during this function's execution
    //assume that the return is an atomic operation...
    return count;
}
double Encoder::getPosition()
{
    return ((double)count * 100.0 / (double)max);
}
void Encoder::zero()
{
    count = 0;
}
void Encoder::setMax(uint32_t _max)
{
    max = _max;
}