#include "DRV8873.h"
#include "Arduino.h"
#include "config.h"

DRV8873 g_drv;

void DRV8873::init()
{
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(FAULT, INPUT_PULLUP); //drv8873 IC provides an open-collector fault signal
}

void DRV8873::setSpeed(int8_t _speed)
{
    if (_speed > 100)
    {
        _speed = 100;
    }
    else if (_speed < -100)
    {
        _speed = -100;
    }
    speed = _speed;
    //scale abs(speed) from 0-100 to 0-255 analogWrite() range
    // subtract from 255, because the drv8873 pins are active LOW
    uint8_t pwm = 255 - (uint8_t)(abs(speed) * (255.0 / 100.0));
    if (speed < 0)
    {
        //reverse
        //PWMA HIGH
        //PWMB goes low to activate
        digitalWrite(PWMA, HIGH);
        analogWrite(PWMB, pwm);
    }
    else if (speed > 0)
    {
        //forward
        //PWMA goes low to activate
        //PWMB HIGH
        analogWrite(PWMA, pwm);
        digitalWrite(PWMB, HIGH);
    }
    else
    {
        //both PWM pins HIGH to brake
        digitalWrite(PWMA, HIGH);
        digitalWrite(PWMB, HIGH);
    }
}

int8_t DRV8873::getSpeed()
{
    return speed;
}

uint16_t DRV8873::readCurrentdA()
{
    return analogRead(CURRENT_SENSE) >> 2; //wrong!
}

uint8_t DRV8873::isInFault()
{
    return !digitalRead(FAULT); //active LOW
}