#pragma once
#include <Arduino.h>

//I2C
const uint8_t I2C_ADDR = 0x44;
const uint32_t CMD_TIMEOUT_PERIOD = 2000;

const uint8_t DISCONNECT_GOAL_POS = 50;
const uint8_t OVERCURRENT_LEVEL_DA = 0;
const double OVERPRESSURE_LEVEL_DBAR = 0;
const double UNDERPRESSURE_LEVEL_DBAR = 0;

//PID
const uint32_t LOOP_PERIOD_US = 10000;
const double KP_POS = 100, KI_POS = 0, KD_POS = 0;
const double KP_PRESS = 1, KI_PRESS = 0, KD_PRESS = 0;
const uint32_t ENCODER_MAX_COUNT = 1000;

//Motors
#define FUEL_PRESS_ENCODER_MAX 100
#define OX_MAIN_ENCODER_MAX 100

//LED flash pattern
#define LED_DISCONNECTED 0b00000001
#define LED_CONNECTED 0b00001111

//Pins
#define LED 13

#define FUEL_PRESS_ENCA 2
#define FUEL_PRESS_ENCB 4
#define FUEL_PRESS_DIR 5
#define FUEL_PRESS_PWM 6
#define FUEL_PRESS_LIMIT 7

#define OX_MAIN_ENCA 2
#define OX_MAIN_ENCB 4
#define OX_MAIN_DIR 5
#define OX_MAIN_PWM 6
#define OX_MAIN_LIMIT 7

#define ENGINE_VENT_SOL 1
#define FUEL_MAIN_SOL 1