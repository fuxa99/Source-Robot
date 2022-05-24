//
// Created by tym14 on 16.03.22.
//
#include <iostream>

#include "Comm.h"
void Comm::Setup(const std::string &remoteip) {
    ip_ = remoteip;
    udp.bind(5468);
}
void Comm::Send(MSG m) {
    udp.sendStr(ip_, NMEA::Build(m));
}

void Comm::loop()
{
    if (!udp.available())
        return;

    auto [s, text] = udp.receiveStr();
    auto msg = NMEA::Parse(text);

    // přijatá zpráva je vadná ?
    if (msg.empty())
        return;

    auto fn = Map.find(msg[0]);
    if (fn != Map.end())
        fn->second(msg);
    //else
        //std::cout<<msg[0] << "není implementován"<<std::endl;
}




