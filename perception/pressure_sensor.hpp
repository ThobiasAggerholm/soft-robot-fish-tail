#ifndef __PRESSURESENSOR__HPP__
#define __PRESURESENSOR__HPP__

//Arduino libraries
#include "Arduino.h"

namespace ps_sensors 
{
    class MPX5100
    {
        public:
        MPX5100();
        void init(uint8_t input_pin);
        float read_pressure();

        private:
        //Pressure sensor calibration factors  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)  Vout=Vs(P * 0.009 + 0.04),  Vs=5V = 1024,  P = 
        const float SensorOffset = 4.44;  //pressure sensor offset
        const float SensorGain = 0.109;   // pressure sensor proportional relation
        uint8_t input_pin;


    };
}


#endif