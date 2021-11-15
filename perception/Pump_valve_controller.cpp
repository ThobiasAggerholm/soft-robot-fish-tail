#include "Pump_valve_controller.hpp"

 PumpController::Pumpcontroller(){

  }
  void Pumpcontroller::init(int PUMP_1_PIN, int PUMP_1_SPEED,
                            int PUMP_2_PIN, int PUMP_2_SPEED,
                            int VALVE_1_PIN, int VALVE_1_DIR,
                            int VALVE_2_PIN, int VALVE_2_DIR)
  {
    pump1_pin=PUMP_1_PIN;
    pump1_speed=PUMP_1_SPEED;
    pump2_pin=PUMP_2_PIN;
    pump2_speed=PUMP_2_SPEED;
    valve1_pin=VALVE_1_PIN;
    valve1_dir=VALVE_1_DIR;
    valve2_pin=VALVE_2_PIN;
    valve2_dir=VALVE_2_DIR;
    pinMode(PUMP_1_PIN, OUTPUT);
    pinMode(PUMP_2_PIN, OUTPUT);
    pinMode(VALVE_1_PIN, OUTPUT);
    pinMode(VALVE_2_PIN, OUTPUT);
  }

void motor_1_on(int motorspeed)
{
  analogWrite(pump1_speed, motorspeed);   //PWM Speed Control   value
  digitalWrite(pump1_pin,HIGH);
  }

void motor_1_off(void)
{
  analogWrite(pump1_speed, 0);   //PWM Speed Control   value
  digitalWrite(pump1_pin,HIGH);
  }

void motor_2_on(int motorspeed)
{
  analogWrite(pump2_speed, motorspeed);   //PWM Speed Control   value
  digitalWrite(pump2_pin,HIGH);
  }

void motor_2_off(void)
{
  analogWrite(pump2_speed, 0);   //PWM Speed Control   value
  digitalWrite(pump2_pin,HIGH);
  }
  void valve_1_on(void)
{
  analogWrite(valve1_dir, 255);   //PWM Speed Control   value
  digitalWrite(valve1_pin,HIGH);
  }

void valve_1_off(void)
{
  analogWrite(valve1_dir, 0);   //PWM Speed Control   value
  digitalWrite(valve1_pin,HIGH);
  }
  void valve_2_on(void)
{
  analogWrite(valve2_dir, 255);   //PWM Speed Control   value
  digitalWrite(valve2_pin,HIGH);
  }

void valve_2_off(void)
{
  analogWrite(valve2_dir, 0);   //PWM Speed Control   value
  digitalWrite(valve2_pin,HIGH);
  }