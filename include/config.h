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

//LED flash pattern
const uint8_t LED_DISCONNECTED = 0b00000001,
              LED_CONNECTED_BRAKE = 0b00001111,
              LED_CONNECTED_MOVE = 0b01010101;

//Pins
const uint8_t LED = 13,
              ENCA = 2,
              ENCB = 4,
              PWMA = 5,
              PWMB = 6,
              LIMIT = 7,
              FAULT = 8,
              CURRENT_SENSE = A0,
              PRESSURE_SENSE = A1;