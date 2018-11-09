/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *         David Long <xiaokun.long@ufactory.cc>  
 *         Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */
 
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string>
#include <swiftpro/SwiftproState.h>

serial::Serial _serial;             // serial object


void processstr(swiftpro::SwiftproState* state, std::string data)
{
    char str[2048];
    strcpy(str, data.c_str());
    char* pch = strtok(str, " ");
    float x, y, z, r;
    int vals = 0;


    while (vals < 4)
    {
        if (pch == NULL)
        {
            ROS_INFO_STREAM("Serial data misaligned. "
                "Defaulting to previous measurement.");
            return;
        }
        switch (pch[0])
        {
            case 'X':
                x = atof(pch+1);
                ++vals;
                break;
            case 'Y':
                y = atof(pch+1);
                ++vals;
                break;
            case 'Z':
                z = atof(pch+1);
                ++vals;
                break;
            case 'R':
                r = atof(pch+1);
                ++vals;
                break;
            case '@':
                break;
            default:
                ROS_INFO_STREAM("Serial data misaligned. "
                    "Defaulting to previous measurement.");
                return;
        }
        pch = strtok(NULL, " ");
    }

    state->motor_angle4 = r;
    state->x = x;
    state->y = y;
    state->z = z;
}


/* 
 * Node name:
 *   swiftpro_read_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   SwiftproState
 */
int main(int argc, char** argv)
{   
    ros::init(argc, argv, "swiftpro_read_node");
    ros::NodeHandle nh;
    swiftpro::SwiftproState swiftpro_state;
    std::string result;

    ros::Publisher pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 1);
    ros::Rate loop_rate(20);

    try
    {
        _serial.setPort("/dev/ttyACM0");
        _serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        _serial.setTimeout(to);
        _serial.open();
        ROS_INFO_STREAM("Port has been open successfully");
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    
    if (_serial.isOpen())
    {
        ros::Duration(3.0).sleep();             // wait 3s
        _serial.write("M2019\r\n");             // detach
        ros::Duration(0.5).sleep();             // wait 0.5s
        _serial.write("M2120 V0.05\r\n");       // report position per 0.05s
        ROS_INFO_STREAM("Start to report data");
    }

    // Default values because these are not used by the uSwift.
    swiftpro_state.pump = 0;
    swiftpro_state.gripper = 0;
    swiftpro_state.swiftpro_status = 0;
    swiftpro_state.motor_angle1 = 0.0;
    swiftpro_state.motor_angle2 = 0.0;
    swiftpro_state.motor_angle3 = 0.0;
    
    while (ros::ok())
    {
        if (_serial.available())
        {
            result = _serial.read(_serial.available());
            ROS_INFO_STREAM("Read:" << result);

            // _serial is reading stuff like "@3 X59.81 Y97.40 Z40.25 R102.00"
            // so there are 5 values, and the last 4 are positions. 
            // The inverse kinematics are calculated _on the uSwift!_

            // my new function
            processstr(&swiftpro_state, result);

            pub.publish(swiftpro_state);

            ROS_INFO(
                "position: X%.2f Y%.2f Z%.2f R%.2f",
                swiftpro_state.x, 
                swiftpro_state.y, 
                swiftpro_state.z, 
                swiftpro_state.motor_angle4
            );
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


