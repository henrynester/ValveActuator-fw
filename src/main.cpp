#include <Arduino.h>

#include <nI2C.h>
#include <AutoPID.h>
#include <util/atomic.h>

#include "DRV8873.h"
#include "Encoder.h"
#include "config.h"
#include "packets.h"

uint32_t t_last_loop_us;
uint32_t t_last_goal_cmd;
uint32_t t_homing_start;

uint8_t stopped_pos; //position of motor when it was last braked

//I2C
CTWI g_twi;
void onI2CRx(const uint8_t data[], const uint8_t length);
void onI2CTx(void);
bool connected;
uint8_t last_cmd;

SensorData_t sensorData;
uint8_t sensor_tx[sizeof(SensorData_t)]; //idk if this one needs to be volatile - the ISR only acceses it. can't hurt
volatile ActuatorData_t actuatorData;

extern DRV8873 g_drv; //motor driver
extern Encoder g_enc; //encoder

double pos, goal_pos, speed;
AutoPID pos_PID(&pos, &goal_pos, &speed, -100, 100, KP_POS, KI_POS, KD_POS);

double readPressure();

uint8_t led_mask = 0b00000001;
uint8_t loop_count = 0;

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); //if LED stays on, we're hanging in setup() somewhere

  //configure this device as an i2c slave
  g_twi.SetLocalDeviceAddress(I2C_ADDR);
  g_twi.SetSlaveReceiveHandler(onI2CRx);
  g_twi.SetSlaveTransmitHandler(onI2CTx);
  g_twi.SetTimeoutMS(1000);
  g_twi.SetSpeed(CTWI::Speed::FAST);

  //configure pins for driver and encoder
  g_drv.init();
  g_enc.init();
  g_enc.setMax(ENCODER_MAX_COUNT);

  //configure PID
  pos_PID.setBangBang(20); //use bang bang control if farther from goal than this

  //Serial.begin(125000);
  //Serial.println("slave");

  delay(500);

  digitalWrite(LED, LOW);
  delay(10);
}

void loop()
{
  if ((micros() - t_last_loop_us) > LOOP_PERIOD_US) //main loop runs at high freq. (~100Hz)
  {
    t_last_loop_us = micros(); //first thing!

    //I2C interrupts cannot occur during copying received data from goal_rx struct to status.goal struct
    //otherwise, part of status.goal would contain data from before the interrupt, and part would contain new data from after the interrupt
    //nasty undefined behavior ensues
    //basically this wrapper makes sure all the code inside executes "like it were one instruction" (of course it is really many)
    //supposedly ATOMIC_RESTORESTATE makes sure the interrupt eventually runs once the block is done. I'm skeptical.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      //sensorData.goal_pos = actuatorData.goal_pos;
    }

    if (connected)
    {
      //if no comms for a certain amount of time
      if (millis() - t_last_goal_cmd > CMD_TIMEOUT_PERIOD)
      {
        connected = false;
      }
    }
    else
    {
      //if connection is lost, direct valve to a known safe position
      sensorData.goal_pos = DISCONNECT_GOAL_POS;
    }

    //the pos_PID stores pointers to doubles pos and goal_pos as inputs, and speed as output
    //here we connect these doubles to the struct
    pos = g_enc.getPosition();
    sensorData.pos = (int8_t)pos;
    goal_pos = (double)sensorData.goal_pos;

    sensorData.current_dA = g_drv.readCurrentdA();
    sensorData.speed = g_drv.getSpeed();
    sensorData.homing.limit_switch = g_enc.readLimit();

    //update warning flags
    // status.warnings.overcurrent = status.current_dA > OVERCURRENT_LEVEL_DA;
    sensorData.faults.drv_failsafe = g_drv.isInFault();
    sensorData.faults.homing_timeout = (!sensorData.homing.has_homed) && (millis() > (t_homing_start + 5000));
    sensorData.faults.limit_switch_stuck = (sensorData.homing.limit_switch) && (sensorData.pos != 0);
    sensorData.faults.open_circuit = (sensorData.current_dA == 0) && (sensorData.speed != 0);

    // status.warnings.overpressure = status.pressure_dbar > OVERPRESSURE_LEVEL_DBAR;
    // status.warnings.underpressure = status.pressure_dbar < UNDERPRESSURE_LEVEL_DBAR;

    //use encoder to figure out whether motor is actually rotating
    if (sensorData.speed == 0)
    {
      stopped_pos = sensorData.pos;
    }
    int8_t delta_pos = sensorData.pos - stopped_pos;
    sensorData.faults.motor_stall = (sensorData.speed != 0) && delta_pos == 0;
    sensorData.faults.motor_slip_encoder_drift = (sensorData.speed == 0) && delta_pos != 0;

    if (sensorData.homing.has_homed)
    {
      //normal operation
      //control motor speed to move toward goal position
      pos_PID.run();
      g_drv.setSpeed((int8_t)speed);
    }
    else
    {
      //we need to home the encoder
      //TODO: have valve move away from zero a bit, then close, to ensure full closure at 0-position
      g_drv.setSpeed(-100); //reverse to zero position
      if (sensorData.homing.limit_switch)
      {
        sensorData.homing.has_homed = true;
      }
      if (t_homing_start == 0)
        t_homing_start = millis();
    }

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

      bool led_on;
      if (connected)
      {
        //pick LED flash pattern based on whether motor is moving
        if (sensorData.speed == 0)
          led_on = LED_CONNECTED_BRAKE & led_mask;
        else
          led_on = LED_CONNECTED_MOVE & led_mask;
      }
      else
      {
        led_on = LED_DISCONNECTED & led_mask; //LED flash pattern
      }
      //digitalWrite(LED, led_on);
    }
    loop_count++;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      //memcpy((void *)(&sensor_tx), (void *)(&sensorData), sizeof(SensorData_t));
    }
  }
}

void onI2CRx(const uint8_t data[], const uint8_t length)
{
  //print all recieved bytes
  // Serial.print("rx:");
  // for (int i = 0; i < length; i++)
  // {
  //   Serial.print(data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  if (length <= 0)
    return; //no data. what do you want, bro?

  switch (data[0]) //first byte is a command code
  {
  case SET_ACTUATOR_DATA:
    if (length == (sizeof(ActuatorData_t) + 1)) //plus one for command byte
    {
      //copy packet data (after first command byte) into goal_rx struct
      //memcpy((void *)(&actuatorData), (void *)(&data[1]), sizeof(ActuatorData_t));

      connected = true; //start working again, we just received a cmd
      t_last_goal_cmd = millis();
    }
    break;
  }

  last_cmd = data[0]; //store the command, so we can reply correctly to message requests
}

void onI2CTx(void)
{
  uint8_t i2c_result = 0;

  switch (last_cmd)
  {
  case REQUEST_SENSOR_DATA:
    i2c_result = g_twi.SlaveQueueNonBlocking(sensor_tx, sizeof(SensorData_t));
    break;
  default:
    break;
  }
  //update status with any i2c bus errors that occur on tx
  //sensorData.faults.i2c_fault = i2c_result ? true : false;
  last_cmd = 0;
  /*
  I2C errors:
  0:success
  1:busy
  2:timeout
  3:data too long to fit in transmit buffer
  4:memory allocation failure
  5:attempted illegal transition of state
  6:received NACK on transmit of address
  7:received NACK on transmit of data
  8:illegal start or stop condition on bus
  9:lost bus arbitration to other master
  */
}

double readPressure()
{
  return analogRead(PRESSURE_SENSE) / 500.0;
}