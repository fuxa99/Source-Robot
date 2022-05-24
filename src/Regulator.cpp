//
// Created by tym14 on 06.04.22.
//

#include "Regulator.h"
#include "math.h"

double Regulator::P(double delta_sens, double velocity) {
    return (-6.633*(-delta_sens)*pow(10,-6) + 0.0007443)*Regulator::P_K;
}

double Regulator::PSD(double delta_angle, double velocity) {
    double sum = error.sum();
    double last_err = error.operator[](error.size()-1);
    error = error.shift(1);
    error.operator[](error.size()-1) = delta_angle;

    return PSD_K*(delta_angle+1/PSD_Ti*sum+PSD_Td*(delta_angle-last_err));
}

/*
 * degrees
 */
double Regulator::delta_to_angle(double delta)
{
    //double min = -3404;
    //double max = 3336;

    //y = 0.0051x - 2.368
    //return +0.0051*delta - 2.368;
    return a_line*delta + b_line;
}