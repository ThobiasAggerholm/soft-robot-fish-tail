#include "bendlab_sensor.hpp"


    BendlabsSensor::BendlabsSensor()
    {
        
    }
    void BendlabsSensor::init(int ADS_RESET_PIN, int ADS_INTERRUPT_PIN)
    {
        // Initialization routine for bendlabs sensor


        ads_init_t init;                                // One Axis ADS initialization structure
        init.sps = ADS_100_HZ;                          // Set sample rate to 100 Hz (Interrupt mode)
        init.ads_sample_callback = [](float * sample, uint8_t sample_type){return;};  // Provide callback for new data. Not used in polled mode. Stub function necessary for library compilation 
        init.reset_pin = ADS_RESET_PIN;                 // Pin connected to ADS reset line
        init.datardy_pin = ADS_INTERRUPT_PIN;           // Pin connected to ADS data ready interrupt
        init.addr = 0;                                  // Update value if non default I2C address is assinged to sensor
        ads_ok = ads_init(&init);                 // Initialize ADS hardware abstraction layer, and set the sample rate
        
        // Enable stretch measurements
        ads_stretch_en(true);
        // Start reading data in polled mode
        ads_polled(true);
        // Wait for first sample
        delay(10);
    }

    int BendlabsSensor::get_ads_status()
    {
        return ads_ok;
    }
    
    float BendlabsSensor::read_angle()
    {
        //Defining varialbes for bendlabs sensor data processing
        float sample[2];
        uint8_t data_type;

        // Read data from the one axis ads sensor
        int ret_val = ads_read_polled(sample, &data_type);

        // Check if read was successfull
        if(ret_val == ADS_OK)
        {
            if(data_type == ADS_SAMPLE)
            {
                // Low pass IIR filter
                signal_filter(sample);

                // Deadzone filter
                deadzone_filter(sample);
            }
            return sample[0];
        }
        return NAN;
    }
    
    
    void BendlabsSensor::parse_com_port(char key)                                       // I2C communication decoding
    {
        switch(key)
        {
            case '0':
            // Take first calibration point at zero degrees
            ads_calibrate(ADS_CALIBRATE_FIRST, 0);
            if(partly_configured == false)
            {
                partly_configured = true;
            }
            else
            {
                configured = true;
            }
            break;
            case '9':
            // Take second calibration point at ninety degrees
            ads_calibrate(ADS_CALIBRATE_SECOND, 90);
            if(partly_configured == false)
            {
                partly_configured = true;
            }
            else
            {
                configured = true;
            }
            break;
            case 'c':
            // Restore factory calibration coefficients
            ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
            break;
            case 'r':
            // Start sampling in interrupt mode
            ads_run(true);
            break;
            case 's':
            // Place ADS in suspend mode
            ads_run(false);
            break;
            case 'f':
            // Set ADS sample rate to 200 Hz (interrupt mode)
            ads_set_sample_rate(ADS_200_HZ);
            break;
            case 'u':
            // Set ADS sample to rate to 10 Hz (interrupt mode)
            ads_set_sample_rate(ADS_10_HZ);
            break;
            case 'n':
            // Set ADS sample rate to 100 Hz (interrupt mode)
            ads_set_sample_rate(ADS_100_HZ);
            break;
            case 'b':
            // Calibrate the zero millimeter linear displacement
            ads_calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
            break;
            case 'e':
            // Calibrate the 30 millimeter linear displacement (stretch), Make certain the sensor is at 0 degrees angular displacement (flat)
            ads_calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
            break;
            default:
            break;
        }
    }

    //BendSensor functions for dataprocessing
    void BendlabsSensor::deadzone_filter(float * sample)                            // Bendlabs sensor Deadzone filter
    {
        static float prev_sample[2];
        float dead_zone = 0.75f;

        for(uint8_t i=0; i<2; i++)
        {
            if(fabs(sample[i]-prev_sample[i]) > dead_zone)
            prev_sample[i] = sample[i];
            else
            sample[i] = prev_sample[i];
        }
    }

    void BendlabsSensor::signal_filter(float * sample)                              // Bendlabs signal filter
    {
        static float filter_samples[2][6];

        for(uint8_t i=0; i<2; i++)
        {
        filter_samples[i][5] = filter_samples[i][4];
        filter_samples[i][4] = filter_samples[i][3];
        filter_samples[i][3] = (float)sample[i];
        filter_samples[i][2] = filter_samples[i][1];
        filter_samples[i][1] = filter_samples[i][0];
    
        // 20 Hz cutoff frequency @ 100 Hz Sample Rate
        filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
            0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

        sample[i] = filter_samples[i][0];
        }
    }  
    
