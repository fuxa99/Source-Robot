//
// Created by tym14 on 16.03.22.
//

#ifndef SAMPLE_DRIVE_H
#define SAMPLE_DRIVE_H


#include "NMEA.h"

class Drive {
private:
    double act_speed_left,act_speed_right;
    //odomotrecké údaje posuvu levého a pravého kola, tz budeme říst v odo
    double Rampa(double wanted_speed, double curr_speed);

public:
    void ODO(MSG item);
    MSG BuildODO();
    MSG BuildSpeed(double v, double w);
    double delta_left, delta_right;
    double speed_left,speed_right;
    double trace_length;
};


#endif //SAMPLE_DRIVE_H
