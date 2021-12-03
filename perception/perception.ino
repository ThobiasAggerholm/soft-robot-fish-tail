//C++ libraries

//Arduino libraries
#include "Arduino.h"
#include "ads.h"

//User includes
#include "bendlab_sensor.hpp"
#include "pressure_sensor.hpp"
#include "pid.hpp"
//Define the input pins for the sensors

#define ADS_RESET_PIN      (18)           // Bendlabs sensor Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN  (19)           // Bendlabs sensor.  

#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)


BendlabsSensor bend_sensor;
ps_sensors::MPX5100 pressure_sensor;
Controller::PID pid;

void setup()
{
    Serial.begin(115200);             // Starting Serial communication with computer baudrate 115200 bps
    bend_sensor.init(ADS_RESET_PIN, ADS_INTERRUPT_PIN);
    pressure_sensor.init(PRESSURE_SENSOR);
    pid.init(1., 10, 2, 5);
}

void loop()
{
    //Print Bendlabs sensor anglePRESSURE_SENSOR
    if(Serial)
    {
        Serial.println("Bendlabs sensor output: ");
        Serial.println(bend_sensor.read_angle());
        Serial.println("Pressure sensor output: ");
        Serial.println(pressure_sensor.read_pressure());
    }
    // Check for received hot keys on the com port
    if(Serial.available())
    {
        bend_sensor.parse_com_port(Serial.read());
    }
  
    // Delay 5ms to keep 100 Hz sampling rate between bend and stretch, Do not increase rate past 500 Hz.
    delay(500);
}
