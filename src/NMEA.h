//
// Created by martin on 02.03.22.
//
#include <string>
#include <vector>

#ifndef SAMPLE_NMEA_H
#define SAMPLE_NMEA_H

using MSG = std::vector<std::string>;
using NmeaCallback = void(MSG);

class NMEA {
public:
    static std::string CRC(const std::string msg);
    static std::string Build(const MSG msg);
    static MSG Parse(const std::string msg);
};

#endif //SAMPLE_NMEA_H
