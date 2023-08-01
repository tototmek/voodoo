#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"

#include <memory>

int main(int argc, char** argv) {

    ros::init(argc, argv, "voodoo");
    ros::NodeHandle n("~");

    ROS_INFO("voodoo started.");

    ros::Publisher publisher =
        n.advertise<sensor_msgs::JointState>("joint_state", 10);

    ros::Subscriber subscriber = n.subscribe<sensor_msgs::Joy>(
        "joy", 10, [&publisher](const sensor_msgs::Joy::ConstPtr& msg) -> void {
            sensor_msgs::JointState joint_state;
            publisher.publish(joint_state);
        });

    ros::spin();

    return 0;
}