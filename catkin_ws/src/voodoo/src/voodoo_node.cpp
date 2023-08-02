#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include <memory>

#include "voodoo/IO.h"
#include "voodoo/voodoo.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voodoo");
    ros::NodeHandle n("~");

    const auto& [joint_names, joint_ranges, joy_ranges] = voodoo::IO::loadParams(n, "");

    voodoo::Remapper remapper(joint_names, joint_ranges, joy_ranges);

    ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("joint_state", 10);

    const auto callback = [&publisher, &remapper](const sensor_msgs::Joy::ConstPtr& msg) -> void {
        sensor_msgs::JointState joint_state = remapper.remap(msg);
        publisher.publish(joint_state);
    };

    ros::Subscriber subscriber = n.subscribe<sensor_msgs::Joy>("joy", 10, callback);
    ros::spin();

    return 0;
}