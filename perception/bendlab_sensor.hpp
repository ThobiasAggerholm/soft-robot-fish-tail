#ifndef __BENDLABS1AXIS__HPP__
#define __BENDLABS1AXIS__HPP__

#include <math.h>

#include "Arduino.h"
#include "ads.h"



class BendlabsSensor
{
    public:
    BendlabsSensor();
    void init(int ADS_RESET_PIN, int ADS_INTERRUPT_PIN);
    int get_ads_status();
    float read_angle();
    void parse_com_port(char key);

    bool configured = false;
    bool partly_configured = false;

    private:
    void deadzone_filter(float *sample);
    void signal_filter(float *sample);

    
    int ads_ok;

};




#endif
