#include "pressure_sensor.hpp"

ps_sensors::MPX5100::MPX5100()
{

}

void ps_sensors::MPX5100::init(uint8_t input_pin_)
{
    //Arduino libraries
    input_pin = input_pin_;
    pinMode(input_pin_, INPUT);  // Defining sensor inputs for ADC (Analog digital converter)
    return;
}

float ps_sensors::MPX5100::read_pressure()
{
    // read the input on analog pin 0:
    float pressure_sensorValue = (analogRead(input_pin)*SensorGain-SensorOffset); //Do maths for calibration
    return pressure_sensorValue;
}