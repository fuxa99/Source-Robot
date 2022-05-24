//cvd
// Created by tym14 on 06.04.22.
//

#ifndef SAMPLE_REGULATOR_H
#define SAMPLE_REGULATOR_H
#include <valarray>

class Regulator {
    std::valarray<double> error;

    double P_K = 150;

    // Puvodní regulátor
    //double PSD_K = 0.06; 0.65
    //double PSD_Ti = 0.95; 0.8
    //double PSD_Td = 0.1; 0.1

    double PSD_K = 0.65; //0.5
    double PSD_Ti = 0.08; //0.05
    double PSD_Td = 5; // 0.1

public:
    Regulator(): error(20){}
    double P(double delta_sens, double velocity);
    double PSD(double delta_angle, double velocity);
    double delta_to_angle(double delta);
    double a_line = 0;
    double b_line = 0;

};


#endif //SAMPLE_REGULATOR_H
