# soft-robot-fish-tail

To run code make sure to download the bendlabs driver "ads_driver" and put into the arduino library.
The driver can be found at https://github.com/bendlabs/one_axis_ads.
Copy the library/ads_driver into your local Arduino/library folder.
Modify the ads_driver/ads.h file by changing the command ADS_DFU_CHECK(1) to ADS_DFU_CHECK(0)


Arduino: 1.8.16 (Linux), Board: "Arduino Uno"

/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino: In member function 'float BendlabsSensor::read_angle()':
perception:77:16: error: 'nanf' was not declared in this scope
         return nanf();
                ^~~~
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino:77:16: note: suggested alternative: 'tanf'
         return nanf();
                ^~~~
                tanf
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino: In function 'void setup()':
perception:173:13: error: 'RUBBER_SENSOR' was not declared in this scope
     pinMode(RUBBER_SENSOR, INPUT);
             ^~~~~~~~~~~~~
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino:173:13: note: suggested alternative: 'PRESSURE_SENSOR'
     pinMode(RUBBER_SENSOR, INPUT);
             ^~~~~~~~~~~~~
             PRESSURE_SENSOR
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino: In function 'void loop()':
perception:183:9: error: 'bend_sensor' was not declared in this scope
         bend_sensor.read_angle();
         ^~~~~~~~~~~
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino:183:9: note: suggested alternative: 'BendlabsSensor'
         bend_sensor.read_angle();
         ^~~~~~~~~~~
         BendlabsSensor
perception:188:9: error: 'bend_sensor' was not declared in this scope
         bend_sensor.parse_com_port();
         ^~~~~~~~~~~
/home/thobias/Documents/MSc1/SOFT/soft-robot-fish-tail/perception/perception.ino:188:9: note: suggested alternative: 'BendlabsSensor'
         bend_sensor.parse_com_port();
         ^~~~~~~~~~~
         BendlabsSensor
exit status 1
'nanf' was not declared in this scope


This report would have more information with
"Show verbose output during compilation"
option enabled in File -> Preferences.
