#include <Arduino.h>

#include <util/atomic.h>

#include "config.h"
#include "i2c_comms.h"
#include "MotorValve.h"
#include "Solenoids.h"

uint32_t t_last_loop_us, t_last_rxd_control;

uint8_t led_mask = 0b00000001;
uint8_t loop_count = 0;

MotorValve fuel_press_valve(FUEL_PRESS_PWM,
                            FUEL_PRESS_DIR,
                            FUEL_PRESS_ENCA,
                            FUEL_PRESS_ENCB,
                            FUEL_PRESS_LIMIT,
                            FUEL_PRESS_ENCODER_MAX);
MotorValve main_ox_valve(OX_MAIN_PWM,
                         OX_MAIN_DIR,
                         OX_MAIN_ENCA,
                         OX_MAIN_ENCB,
                         OX_MAIN_LIMIT,
                         OX_MAIN_ENCODER_MAX);
Solenoids solenoids;

void runLED();

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); //if LED stays on, we're hanging in setup() somewhere

  initI2C();
  fuel_press_valve.init();
  main_ox_valve.init();
  solenoids.init();

  Serial.begin(125000);
  Serial.println("slave");

  digitalWrite(LED, LOW);
}

void loop()
{
  if ((micros() - t_last_loop_us) > LOOP_PERIOD_US) //main loop runs at high freq. (~100Hz)
  {
    t_last_loop_us = micros(); //first thing!

    //I2C interrupts cannot occur during copying of received control data into the status data
    //otherwise, part of the status data would contain data from before the interrupt, and part would contain new data from after the interrupt
    //nasty undefined behavior ensues
    //basically this wrapper makes sure all the code inside executes "like it were one instruction" (of course it is really many)
    //supposedly ATOMIC_RESTORESTATE makes sure the interrupt eventually runs once the block is done. I'm skeptical.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      status.fuel_press_valve.goal_pos = control_rx.fuel_press_valve_goal_pos;
      status.main_ox_valve.goal_pos = control_rx.main_ox_valve_goal_pos;
      status.solenoids = control_rx.solenoids;
    }

    if (just_rxd_control)
    {
      t_last_rxd_control = millis();
      connected = true;
      just_rxd_control = false;
    }

    if (connected)
    {
      //if no comms for a certain amount of time
      if (millis() - t_last_rxd_control > CMD_TIMEOUT_PERIOD)
      {
        connected = false;
      }
    }
    else
    {
      //if connection is lost, direct valves to a known safe position:
      status.fuel_press_valve.goal_pos = 0;             //don't pressurize fuel tank
      status.main_ox_valve.goal_pos = 0;                //don't let ox into engine
      status.solenoids.engine_vent_valve_close = false; //vent pressurant from fuel tank through engine
      status.solenoids.main_fuel_valve_open = false;    //don't let fuel into engine
    }

    fuel_press_valve.run(&status.fuel_press_valve);
    main_ox_valve.run(&status.main_ox_valve);
    solenoids.run(&status.solenoids);
    runLED();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      memcpy((void *)(&status_tx), (void *)(&status), sizeof(Status_t));
    }
  }
}

void runLED()
{
  //LED flash pattern
  if (loop_count > 10)
  {
    loop_count = 0;
    if (led_mask == 0b10000000)
    {
      led_mask = 0b00000001;
    }
    else
    {
      led_mask = led_mask << 1; //shift the one-bit in the LED flashing mask one left each loop
    }

    bool led_on = (connected ? LED_CONNECTED : LED_DISCONNECTED) & led_mask;
    digitalWrite(LED, led_on);
  }
  loop_count++;
}