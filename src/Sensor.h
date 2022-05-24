//
// Created by tym14 on 16.03.22.
//

#ifndef SAMPLE_SENSOR_H
#define SAMPLE_SENSOR_H


#include "NMEA.h"

class Sensor {
private:

public:
    int sensor[5];
    int sensor_min[5] = {710, 710, 710, 710, 710};
    int sensor_max[5] = {2900, 2900, 2900, 2900, 2900};
    int sensor_on = 0;
    bool SensorState(int mask_on, int mask_off);
    void SENZOR(MSG item);
    MSG BuildSenzor(int i);
};


#endif //SAMPLE_SENSOR_H
