//
// Created by tym14 on 16.03.22.
//

#include "Sensor.h"
#include "iostream"

void Sensor::SENZOR(MSG item) {
    int index = std::stoi(item[1]);
    int val = std::stoi(item[2]);
    if(sensor_min[index] > val) sensor_min[index] = val;
    if(sensor_max[index] < val) sensor_max[index] = val;
    //sensor_on[index] = val > sensor_min[index]+450;
    sensor_on = val > sensor_min[index] + 450 ?
            sensor_on | (1 << index) :
            sensor_on & ~(1 << index);
    sensor[index] = val;
}

MSG Sensor::BuildSenzor(int i) {
    return {"SENSOR",std::to_string(i)};
}

bool Sensor::SensorState(int mask_on, int mask_off){
    return
            ((sensor_on & mask_on) == mask_on) &&
            ((~sensor_on & mask_off) == mask_off);
}