//
// Created by tym14 on 16.03.22.
//
#include "math.h"
#include "Drive.h"

void Drive::ODO(MSG item) {
    double R = 0.02;
    double konst = (2*M_PI*R)/(200*32);

    delta_left = std::stod(item[1]) * konst;
    delta_right = std::stod(item[2]) * konst;
    trace_length += delta_left;
}

MSG Drive::BuildSpeed(double v, double w) {
    double R = 0.02;
    double konst = (200*32)/(2*M_PI*R);
    double C_TW = 0.12;
    double C_L = konst;
    double C_R = konst;

    w*=2*M_PI/360;
    act_speed_left = Rampa((v + 0.5*C_TW*w)*C_L, act_speed_left);
    act_speed_right = Rampa((v - 0.5*C_TW*w)*C_R, act_speed_right);
    return {"SPEED",std::to_string(act_speed_left),std::to_string(act_speed_right)};
}

MSG Drive::BuildODO() {

    return {"ODO"};
}

/*
 * wanted_speed m/s
 * */
double Drive::Rampa(double wanted_speed, double curr_speed) {
    double diff = 120;

    double delta_speed = wanted_speed - curr_speed;

    if(delta_speed > diff)
        delta_speed = diff;
    if(delta_speed < -diff)
        delta_speed = -diff;

    return  curr_speed + delta_speed;
}
