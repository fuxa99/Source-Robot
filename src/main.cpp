#include <roboutils/io/I2C.h>
#include <roboutils/util/timing.h>
#include <ros/ros.h>
#include "RvizExampleClass.h"
#include <fstream>
#include "iostream"
#include "math.h"

#include "Comm.h"
#include "Drive.h"
#include "Sensor.h"
#include "Regulator.h"

using namespace RoboUtils;
using namespace RoboUtils::IO;

bool run{true};
void PONG (MSG)
{
    std::cout<<"OK"<<std::endl;
}

int main(int argc, char* argv[])
{
    Comm comm;
    Drive drive;
    Sensor sensor;
    Regulator regulator;
    long loop_period = 15; // 50ms => 20Hz+

    comm.Setup("127.0.0.1/8080");
    comm.Send({"PING"});
    comm.Send({"RESET"});

    //Namapování zpráv
    comm.Map.insert({"SENSOR",[&](auto a){sensor.SENZOR(a);}});
    comm.Map.insert({"ODO",[&](auto a){drive.ODO(a);}});
    comm.Map.insert({"PONG",[&](auto a){PONG(a);}});

    ros::init(argc, argv, "cool_node_name");
    auto node = ros::NodeHandle();
    auto rviz_visualizer = RvizExampleClass(node, "rviz_topic", 30.0);

    long long ms = RoboUtils::millis() + loop_period;

    // file pointer
    std::fstream f1;

    // opens an existing csv file or creates a new file.
    f1.open("delta.csv", std::ios::out | std::ios::app);
    //f2.open("distance.csv", ios::out | ios::app);

    const int n = 5;
    double w = 0;
    double v = 0;
    bool line_break_bool = false; // proměná pro vypínání krajních senzorů při přerušení
    bool vypis = true;
    int line_break_detect = 1; // odbočení při rozdvojení
    bool line_break_detect_bool = false;
    double distance = 0;
    double delta_distance = 0;

    //bool straight_line_brake = false; pro paralelní čáru při přerušení

    // sett parametrs
    double v_max = 0.10; // max robot speed at straight line
    double w_max = 40; // max w
    double samples_off = 10; // delte last samples from buffer_w
    double engine_brake = 5; // increase variable for better engine brake
    double line_break_detect_distance = 0.6; // distance between line break and branch

    double delta_sens = 1000;
    double diff = 0;
    int state = 3;

    std::valarray<double> buffer_w(45);
    std::valarray<bool> line_break(20);

    rviz_visualizer.delta_sens = &delta_sens;
    rviz_visualizer.w = &w;
    rviz_visualizer.v = &v;
    rviz_visualizer.state = &state;
    rviz_visualizer.mask = &(sensor.sensor_on);

    int kalibrace = 0;

    while (run)
    {
        comm.Send(drive.BuildODO());
        for(int i = 0; i < n; i++)
            comm.Send(sensor.BuildSenzor(i));

        while (RoboUtils::millis() < ms)
            comm.loop();

        ms = RoboUtils::millis() + loop_period;

        if(kalibrace == 1)
        {
            /*
             * 5 SENSORS
             * ROBOT INDEXING : R 4 3 2 1 0 L
             * MASKING        : 4 3 2 1 0
             *
             * POSSIBLE MODES  0 1 1 1 1 || 1 1 1 1 0
             * if
             *  1 1 1 1 1        vertical path disturbance              STATE 1
             *  0 0 0 0 0        line break                             STATE 2
             * else if
             *  X 1 1 1 X        normal mode, 2 differial sensors       STATE 3
             *  1 1 X 0 0        left turn                              STATE 4
             *  1 X 0 0 0        big left turn                          STATE 5
             *  0 0 X 1 1        right turn                             STATE 4
             *  0 0 0 X 1        big right turn                         STATE 5
             *  1 1 0 1 1        path split                             STATE 6
             * else
             *  X X X X X        the world will burn
             */
            if(sensor.SensorState(0, 31)) {
                state = 2;
                line_break_bool = false; // false vypne omezení bočních senzorů při přerušení
                line_break_detect_bool = true;
                line_break_detect = -1;
            }else if(sensor.SensorState(31, 0)){
                state = 1;
            }else if(sensor.SensorState(24, 3) || sensor.SensorState(3, 24)){
                state = 4;
            }else if(sensor.SensorState(16,   7) || sensor.SensorState(1, 28)){
                if (!line_break_bool)
                    state = 5;
            }else if(sensor.SensorState(27, 4) || sensor.SensorState(26, 5) || sensor.SensorState(19, 12) || sensor.SensorState(17, 14) ){
                state = 6;
            }else if(sensor.SensorState(14, 0) || sensor.SensorState(4, 0)){
                state = 3;
                line_break_bool = false;
            }else{
                //ugh
                state = 3;
            }

            //for(int i = 0; i < n; i++)
            //   std::cout << "SENZOR " << i << " " << sensor.sensor[i] << std::endl;;

            switch(state) {
                case 1:
                    //std::cout << "DIS";
                    w = w > 1 ? 1 : w < -1 ? -1 : w;
                    buffer_w = buffer_w.shift(1);
                    buffer_w.operator[](buffer_w.size()-1) = w;
                    break;
                case 2: {
                    // line break
                    double sum = 0;
                    for(int i = 0; i < buffer_w.size() - samples_off; i++)
                        sum += buffer_w[i];
                    w = sum/(buffer_w.size()-samples_off);
                    break;
                }
                case 3:
                    delta_sens =
                            - sensor.sensor[1]
                            + sensor.sensor[3];

                    diff = regulator.delta_to_angle(delta_sens);
                    w = regulator.PSD(diff, v);

                    buffer_w = buffer_w.shift(1);
                    buffer_w.operator[](buffer_w.size()-1) = w;
                    break;
                case 4:
                    if(!sensor.SensorState(24, 3)){
                        delta_sens =
                                (-(sensor.sensor_max[2]-sensor.sensor[2])
                                 - sensor.sensor[1])*0.7;
                    }else{
                        delta_sens =
                                (sensor.sensor_max[2]-sensor.sensor[2]
                                 + sensor.sensor[3])*0.7;
                    }

                    diff = regulator.delta_to_angle(delta_sens);
                    w = regulator.PSD(diff, v);

                    buffer_w = buffer_w.shift(1);
                    buffer_w.operator[](buffer_w.size()-1) = w;
                    break;
                case 5:
                    if(sensor.SensorState(16, 7)){ // left
                        delta_sens =
                                + (sensor.sensor_max[3]-sensor.sensor[3])
                                + sensor.sensor[4];
                    }else{ // right
                        delta_sens =
                                - (sensor.sensor_max[1]-sensor.sensor[1])
                                - sensor.sensor[0];
                    }

                    diff = regulator.delta_to_angle(delta_sens);
                    w = regulator.PSD(diff, v);

                    buffer_w = buffer_w.shift(1);
                    buffer_w.operator[](buffer_w.size()-1) = w;
                    break;
                case 6:
                    // line split
                    delta_sens = -3000*(line_break_detect);  // linebreak ==> snažší při detekci pravá
                            //- sensor.sensor[4]
                            //+ sensor.sensor[3];

                    diff = regulator.delta_to_angle(delta_sens);
                    w = regulator.PSD(diff, v);

                    buffer_w = buffer_w.shift(1);
                    buffer_w.operator[](buffer_w.size()-1) = w;

                    if(sensor.sensor_max[1]*0.9 < sensor.sensor[1])
                         state = 1;
                    break;
                default:
                    break;
            }

            if (line_break_detect < 0){
                if (line_break_detect_bool) {
                    delta_distance = drive.trace_length;
                    distance = 0;
                    line_break_detect_bool = false;
                }
                distance = drive.trace_length - delta_distance;
                if (distance > line_break_detect_distance){
                    line_break_detect = 1;
                }

            }
            //v = 0.03/abs(w);
            //v = v < 0.01 ? 0.01 : v > 0.08 ? 0.08 : v;


            w = w < -w_max ? -w_max : w > w_max ? w_max : w;

            v = v_max-abs(w*v_max/(w_max-engine_brake));
            if (v < 0)
                v = 0;

            /*v = 0; w = 36;
            for(int i = 0; i < n; i++)
                f1 << sensor.sensor[i] << ';';
            f1<<std::endl;
            f1.flush();*/

            comm.Send(drive.BuildSpeed(v, w));

            rviz_visualizer.draw_predicted_trajectory(v,-w*M_PI/180);
        }else{
            if(abs(drive.trace_length) < 0.36 ){
                if(abs(drive.trace_length) < 2)
                    comm.Send(drive.BuildSpeed(0, -80));
                else
                    comm.Send(drive.BuildSpeed(0, -5));
            }
            else{
                comm.Send(drive.BuildSpeed(0, 0));
                regulator.a_line = 0.36/(sensor.sensor_max[1]+sensor.sensor_max[3]);
                regulator.b_line = 0.18 - sensor.sensor_max[3]*regulator.a_line;
                kalibrace = drive.delta_left == 0;
                if(kalibrace == 0 && vypis){
                    std::cout << "Sensory max hodnoty:" << sensor.sensor_max[1]<< ":" << sensor.sensor_max[3] << std::endl;
                    std::cout << "y = " << regulator.a_line<< "*x + " << regulator.b_line << std::endl;
                    vypis = false;
                }
            }

        }
        ros::spinOnce();

    }
}