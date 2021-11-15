#ifndef __PUMPC_VALVE_CONTROLLER__HPP__
#define __PUMPC_VALVE_ONTROLLER__HPP__
#include "Arduino.h"



class PumpValveController
{
    public:
    PumpController();
    init(int PUMP_1_PIN, int PUMP_1_SPEED,
         int PUMP_2_PIN, int PUMP_2_SPEED,
         int VALVE_1_PIN, int VALVE_2_PIN);
    void pump_1_on(int motorspeed);
    void pump_1_off();
    void pump_2_on(int motorspeed);
    void pump_2_off();
    void valve_1_on();
    void valve_1_off();
    void valve_2_on();
    void valve_2_off();

    int pump1_pin,pump1_speed,pump2_pin,pump2_speed,valve1_pin,valve1_dir,valve2_pin,valve2_dir;

};


#endif
