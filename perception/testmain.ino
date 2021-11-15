#include "Arduino.h"
#include "Pump_valve_controller.hpp"


//Arduino PWM Speed Control：
int PUMP_1_SPEED = 3;    ///<Pump 1 Speed
int PUMP_1_PIN = 4;    ///<Pump 1 Direction

int VALVE_1_PIN = 11;   ///<Valve 1 Enable
int VALVE_1_DIR = 12;  ///<Valve  1 Direction

int PUMP_2_SPEED = 5; ///<Pump 2 Speed
int PUMP_2_PIN = 8; ///<Pump 2 Direction

int VALVE_2_PIN = 6; ///<Valve 2 Enable
int VALVE_2_DIR = 7; ///<Valve 1 Direction

int timecounter = 1;  // Auxiliar variable for controlling the time of the process

PumpValveController PVcontroller;

void setup()
{
  PVcontroller::init(PUMP_1_PIN, PUMP_1_SPEED,
         PUMP_2_PIN, PUMP_2_SPEED,
         VALVE_1_PIN, VALVE_1_DIR,
         VALVE_2_PIN, VALVE_2_DIR);
}

void loop()
{
  int motorspeed=100;      

  // Introducion air to the PneuNets
  pump_1_on(250);
  valve_1_on();

  pump_2_on(150);
  valve_2_on();

  //Stop pump
  pump_1_off();
  pump_2_off();

  // Release the air
  valve_1_off();
  valve_2_off();

  //Controlling the time of the process
  if (timecounter <= 30)     // 3 seconds == 30 * 100 ms
  {
   // Introducion air to the PneuNets
   motor_1_on(250);
   valve_1_on();

   timecounter++;
  }
  else if ( timecounter > 30 and timecounter<= 60 )
  {
    motor_1_off();
    valve_1_on();
    timecounter++;
  }
  else if ( timecounter > 60 and timecounter<= 90)
  {
    motor_1_off();
    valve_1_off();
    timecounter++;
  }
  else if ( timecounter > 90)
  {
    timecounter=0;
  }
      
  delay(100);   // defining sample time = 100 miliseconds
}