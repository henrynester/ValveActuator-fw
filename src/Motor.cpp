#include "Motor.h"
#include "Arduino.h"

void Motor::init()
{
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
}

void Motor::setSpeed(int8_t speed)
{
    if (speed > 100)
    {
        speed = 100;
    }
    else if (speed < -100)
    {
        speed = -100;
    }
    //scale abs(speed) from 0-100 to 0-255 analogWrite() range
    uint8_t pwm = (uint8_t)(abs(speed) * (255.0 / 100.0));

    //speed = 0 -> pwm = 0 -> both poles of motor tied low -> motor brakes
    analogWrite(pwm_pin, pwm);

    //dir_pin is high to rotate one way and low to rotate the other way
    digitalWrite(dir_pin, (speed > 0) ? HIGH : LOW);
}