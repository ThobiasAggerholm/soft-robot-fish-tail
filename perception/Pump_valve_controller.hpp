#ifndef __PUMP_VALVE_CONTROLLER__HPP__
#define __PUMP_VALVE_CONTROLLER__HPP__
#include "Arduino.h"



class PumpValveController
{
    public:
    PumpValveController();
    void init(int PUMP_1_PIN, int PUMP_1_SPEED,
         int PUMP_2_PIN, int PUMP_2_SPEED,
         int VALVE_1_PIN, int VALVE_1_DIR,
         int VALVE_2_PIN, int VALVE_2_DIR);
    void pump_1_on(int motorspeed);
    void pump_1_off(void);
    void pump_2_on(int motorspeed);
    void pump_2_off(void);
    void valve_1_on(void);
    void valve_1_off(void);
    void valve_2_on(void);
    void valve_2_off(void);

    int pump1_pin,pump1_speed,pump2_pin,pump2_speed,valve1_pin,valve1_dir,valve2_pin,valve2_dir;

};


#endif
