
//C++ libraries

//Arduino libraries
#include "Arduino.h"
#include "ads.h"


#include "math.h"

//User includes
#include "bendlab_sensor.hpp"
#include "pressure_sensor.hpp"
#include "pid.hpp"
#include "Pump_valve_controller.hpp"
//Define the input pins for the sensors

#define ADS_RESET_PIN      (18)           // Bendlabs sensor Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN  (19)           // Bendlabs sensor.  

#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)


BendlabsSensor bend_sensor;
ps_sensors::MPX5100 pressure_sensor;
Controller::PID pid;
PumpValveController pump_valve_controller;

//Arduino PWM Speed Control：
int E1 = 3;    ///<Pump 1 Speed
int M1 = 4;    ///<Pump 1 Direction

int E2 = 11;   ///<Valve 1 Enable
int M2 = 12;  ///<Valve  1 State

const int E3 = 5; ///<Pump 2 Speed
const int M3 = 8; ///<Pump 2 Direction

const int E4 = 6; ///<Valve 2 Enable
const int M4 = 7; ///<Valve 1 Direction

bool configured = false;

float sin_x = 0;
float sin_w = 1./8.;
float sin_A = 20.;



void setup()
{
    Serial.begin(115200);             // Starting Serial communication with computer baudrate 115200 bps
    bend_sensor.init(ADS_RESET_PIN, ADS_INTERRUPT_PIN);
    pressure_sensor.init(PRESSURE_SENSOR);
    pid.init(1, 5, 1, 5);
    pump_valve_controller.init(M1, E1, M3,E3, M2, E2,
         M4, E4);
    pump_valve_controller.valve_2_off();
    pump_valve_controller.valve_1_off();
    bend_sensor.parse_com_port('c');

}

void loop()
{
  /*
  delay(3000);
  while(true)
  {
     pump_valve_controller.valve_1_off();
     pump_valve_controller.valve_2_on();
     pump_valve_controller.pump_1_on(100);
     delay(2000);
     pump_valve_controller.valve_2_off();
     pump_valve_controller.valve_1_on();
     pump_valve_controller.pump_2_on(100);
     delay(2000);   
  }*/

   
   
    if(bend_sensor.configured)
    {
        float bend_sensor_angle = bend_sensor.read_angle();
        if(abs(bend_sensor_angle) < 180 && abs(bend_sensor_angle) > 0.001)
        {

          float target_angle = sin_A * sin(sin_w*sin_x);
          
          Serial.print(target_angle);
          Serial.print(",");
          Serial.println(bend_sensor_angle);


 
          
          if(target_angle >= 0)
          {
            float motor_command = pid.calculate(target_angle, bend_sensor_angle);
            pump_valve_controller.valve_2_on();
            if(motor_command >= 0)
            {
                pump_valve_controller.valve_1_off();
                pump_valve_controller.pump_1_on(motor_command);
               
            }
            else
            {
                //pump_valve_controller.pump_2_on(-motor_command);
                pump_valve_controller.valve_1_on();
            }
            
          }
          else
          {
            float motor_command = pid.calculate(-target_angle, -bend_sensor_angle);
            pump_valve_controller.valve_1_on();
            if(motor_command >= 0)
            {
                pump_valve_controller.valve_2_off();
                pump_valve_controller.pump_2_on(motor_command);
                
            }
            else
            {
                //pump_valve_controller.pump_2_on(-motor_command);
                pump_valve_controller.valve_2_on();
            }            
          }
          

          
          //Serial.println(motor_command);
  
  

  
           
                  
          }
          sin_x += 0.1;
    }
    else{
      pump_valve_controller.valve_2_on();
      pump_valve_controller.valve_1_on();
    }
        


    // Check for received hot keys on the com port
    if(Serial.available())
    {
        bend_sensor.parse_com_port(Serial.read());
    }

    
  
    // Delay 5ms to keep 100 Hz sampling rate between bend and stretch, Do not increase rate past 500 Hz.
    delay(100);
    

}
