#pragma once

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

class RosExampleClass {

public:
    RosExampleClass(ros::NodeHandle& node, const std::string& topic, const float freq) : node_{node} {
        publisher_ = node.advertise<std_msgs::Float32>(topic, 0);
        subscriber_ = node.subscribe(topic, 0, &RosExampleClass::subscriber_callback, this);
        timer_ = node.createTimer(freq, &RosExampleClass::timer_callback, this);
        start_time_ = ros::Time::now();
    }

private:

    void timer_callback(const ros::TimerEvent& event) const {
        std::cout << "Timer callback called" << std::endl;
        auto uptime = (ros::Time::now() - start_time_).toSec();
        publish_message(uptime);
    }

    void subscriber_callback(const std_msgs::Float32 msg) const {
        std::cout << "Just received: " << msg << std::endl;
    }

    void publish_message(float value_to_publish) const {
        std_msgs::Float32 msg;
        msg.data = value_to_publish;
        publisher_.publish(msg);
        std::cout << "Just sent: " << msg.data << std::endl;
    }

    ros::NodeHandle &node_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Timer timer_;
    ros::Time start_time_;
};