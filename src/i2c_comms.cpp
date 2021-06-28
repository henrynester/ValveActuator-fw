#include <Arduino.h>
#include <Wire.h>
#include "i2c_comms.h"
#include "config.h"

void initI2C()
{
    //configure this device as an i2c slave
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Wire.begin(I2C_ADDR);
    //DISABLE pullups to 5V on I2C lines
    //in case we want to connect to a 3.3V device
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
}

void readArrayI2C(uint8_t array[], const uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        array[i] = Wire.read();
    }
}

void onI2CReceive(int num_bytes)
{
    // while (Wire.available())
    // {
    //   Serial.print(Wire.read(), HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    if (!Wire.available())
        return;

    uint8_t cmd = Wire.read();
    //Serial.println(cmd);

    switch (cmd)
    {
    case CMD_CONTROL:
    {
        if (num_bytes != sizeof(Control_t) + 1)
        {
            break;
        }
        readArrayI2C((uint8_t *)&control_rx, sizeof(Control_t));
        last_cmd = CMD_CONTROL;
        just_rxd_control = true;
    }
    break;
    case CMD_STATUS:
        last_cmd = CMD_STATUS;
        break;
    default:
        last_cmd = CMD_NONE;
        break;
    }

    while (Wire.available())
    {
        Wire.read();
    }
}

void writeArrayI2C(const uint8_t array[], const uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        Wire.write(array[i]);
    }
}

void onI2CRequest(void)
{
    switch (last_cmd)
    {
    case CMD_STATUS:
        writeArrayI2C(status_tx, sizeof(Status_t));
        break;
    default:
        break;
    }
    last_cmd = CMD_NONE;
}

bool connected, just_rxd_control;
enum I2CCommand_t last_cmd;

struct Status_t status;
uint8_t status_tx[sizeof(struct Status_t)]; //idk if this one needs to be volatile - the ISR only acceses it. can't hurt

struct Control_t control_rx;