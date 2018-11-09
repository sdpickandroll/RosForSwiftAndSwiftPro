/* 
 * Author: Joshua Petrin  <lol@vanderbilt.edu>      
 */
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/position.h>
#include <swiftpro/angle4th.h>

#include <swiftpro/position_change.h>






int main(int argc, char** argv)
{   
    ros::init(argc, argv, "swiftpro_bare_bones_node");
    ros::NodeHandle nh;
    swiftpro::SwiftproState swiftpro_state;

    ros::Subscriber reader_sub = nh.subscribe("SwiftproState_topic");

    ros::Subscriber sub1 = nh.subscribe("position_write_topic", 1, position_write_callback);
    ros::Subscriber sub2 = nh.subscribe("swiftpro_status_topic", 1, swiftpro_status_callback);
    ros::Subscriber sub3 = nh.subscribe("angle4th_topic", 1, angle4th_callback);
    ros::Subscriber sub4 = nh.subscribe("gripper_topic", 1, gripper_callback);
    ros::Subscriber sub5 = nh.subscribe("pump_topic", 1, pump_callback);
    ros::Publisher   pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 1);
    ros::Rate loop_rate(20);

    try
    {
        _serial.setPort("/dev/ttyACM0");
        _serial.setBaudrate(115200);
        _serial.setTimeout(serial::Timeout::simpleTimeout(1000));
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
        ros::Duration(3.5).sleep();             // wait 3.5s
        _serial.write("M2120 V0\r\n");          // stop report position
        ros::Duration(0.1).sleep();             // wait 0.1s
        _serial.write("M17\r\n");               // attach
        ros::Duration(0.1).sleep();             // wait 0.1s
        ROS_INFO_STREAM("Attach and wait for commands");
    }

    while (ros::ok())
    {
        pub.publish(pos);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
