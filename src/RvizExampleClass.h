#pragma once
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "math.h"

#define format std::fixed << std::setw(5) << std::showpos << std::setprecision(2)

class RvizExampleClass {

    class Pose {
    public:
        Pose(float x, float y, float z) : x_{x}, y_{y}, z_{z} {}
        float x() const {return x_;}
        float y() const {return y_;}
        float z() const {return z_;}
    private:
        const float x_, y_, z_;
    };

public:
    double * w, * v, * delta_sens;
    int* state, *mask;
    RvizExampleClass(ros::NodeHandle& node, const std::string& topic, float freq): node_{node} {
        timer_ = node.createTimer(freq, &RvizExampleClass::timer_callback, this);
        markers_publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 0);
        trajectory_publisher_ = node.advertise<visualization_msgs::Marker>("trajectory", 0);
    }

    void draw_predicted_trajectory(float lin_vel, float ang_vel) {

        // Create message instance
        auto msg = visualization_msgs::Marker{};

        // Define frame and timestamp
        msg.header.frame_id = "robot"; // frame "robot" means, that (0,0) point is in the origin of your robot
        msg.header.stamp = ros::Time().now();
        msg.id = 0;

        // Define marker type
        msg.type = visualization_msgs::Marker::LINE_STRIP; // lines that interconnect set of points
        msg.action = visualization_msgs::Marker::ADD;

        // Define
        msg.pose.orientation.w = 1.0; // avoid invalid quaternion
        msg.scale.x = 0.005; // line strip width in meters

        // Purple color
        msg.color.b = 1.0;
        msg.color.r = 1.0;
        msg.color.a = 1.0;

        // First point (robot origin)
        geometry_msgs::Point previous_p;
        previous_p.x = 0.0;
        previous_p.y = 0.0;
        previous_p.z = 0.0;

        float theta = 0.0f;
        float dt = 0.1;     // trajectory approximation step
        for (size_t i = 0; i < 50; ++i){    // 0.1s * 50 = 5s of prediction
            geometry_msgs::Point p = previous_p;

            // Arc trajectory
            if (ang_vel != 0) {
                float radius = lin_vel / ang_vel;
                p.x += -radius * sin(theta) + radius * sin(theta + ang_vel*dt); // see the arc trajectory model
                p.y +=  radius * cos(theta) - radius * cos(theta + ang_vel*dt);
                theta += ang_vel*dt;
            }
                // Directly forward
            else {
                p.x = previous_p.x + lin_vel * dt;
            }

            // Add trajectory point to message
            msg.points.push_back(p);
            previous_p = p;
        }

        // Send trajectory to RViz
        trajectory_publisher_.publish(msg);
    }

private:

    void timer_callback(const ros::TimerEvent& event) {
        auto time = ros::Time::now().toSec();
        auto pose = Pose(1, 1, 0.2);

        visualization_msgs::MarkerArray msg;

        msg.markers.push_back(make_text_marker(pose));
        markers_publisher_.publish(msg);
    }

    visualization_msgs::Marker make_text_marker(const Pose& pose) {
        visualization_msgs::Marker text;

        // Coordination system
        text.header.frame_id = "origin";

        // Timestamp
        text.header.stamp = ros::Time();

        // Marker Type
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.id = 1;

        // Position
        text.pose.position.x = pose.x();
        text.pose.position.y = pose.y();
        text.pose.position.z = pose.z() + 0.3;

        // Size
        text.scale.z = 0.1;

        // Text
        std::stringstream stream;
        stream << " w : " << round(*w*100)/100 << std::endl
            << " v : " << round(*v*1000)/1000 << std::endl
            << " d : " << *delta_sens << std::endl
            << " s : " << *state << std::endl
            << " m : " << *mask << std::endl;
        /*<< "  x: " << format << pose.x() << std::endl
               << "  y: " << format << pose.y() << std::endl
               << "  z: " << format << pose.z();*/
        text.text = stream.str();

        // Color
        text.color.a = 1.0; // alpha - visibility
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 0.0;
        return text;
    }

    ros::NodeHandle& node_;
    ros::Timer timer_;
    ros::Publisher markers_publisher_;
    ros::Publisher trajectory_publisher_;
};