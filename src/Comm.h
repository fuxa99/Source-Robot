//
// Created by tym14 on 16.03.22.
//

#ifndef SAMPLE_COMM_H
#define SAMPLE_COMM_H

#include <map>
#include "functional"
#include "NMEA.h"
#include "roboutils/comm/UDP.h"

class Comm {
private:
    RoboUtils::COMM::UDP udp;
    std::string ip_;
public:
    void Setup(const std::string &remoteip);
    void loop();
    void Send(MSG m);

    std::map<std::string, std::function<NmeaCallback>> Map;
};


#endif //SAMPLE_COMM_H
