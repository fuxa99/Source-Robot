//
// Created by martin on 02.03.22.
//

#include "NMEA.h"
#include "roboutils/util/strings.h"

std::string NMEA::CRC(const std::string msg) {
    uint8_t CRC{};
    for(auto i:msg){
        CRC^=i;
    }
    const char* p{"0123456789ABCDEF"};
    return{p[(CRC>>4)&0x0F],p[CRC&0x0F]};
}

std::string NMEA::Build(const MSG msg) {
    std::string text = RoboUtils::join(msg,",");
    return "$"+text+"*"+CRC(text);
}

MSG NMEA::Parse(const std::string msg) {
    auto begin = msg.find('$');
    if (begin == std::string::npos)
        return{};
    auto end = msg.find('*');
    std::string mess = msg.substr(begin+1,end-begin-1);
    std::string control = msg.substr(end+1,2);

    if (CRC(mess) != msg.substr(end+1,2))
        return {};
    return RoboUtils::split(mess,',');
}
