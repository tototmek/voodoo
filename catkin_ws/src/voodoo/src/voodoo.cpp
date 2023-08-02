#include "voodoo/voodoo.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include <string>
#include <vector>

namespace voodoo {

namespace {

double remapRange(const double in, Range<double> in_range, Range<double> out_range)
{
    const double t = (in - in_range.minimum) / (in_range.span);
    return out_range.minimum + t * (out_range.span);
}

}  // namespace

Remapper::Remapper(const std::vector<std::string>& names,
                   const std::vector<Range<double>>& joint_ranges,
                   const std::vector<Range<double>>& joy_ranges)
    : n_channels_(names.size())
    , names_(names)
    , joint_ranges_(joint_ranges)
    , joy_ranges_(joy_ranges)
{
    if (joint_ranges_.size() != names_.size()) {
        ROS_ERROR("Joint name count and joint range count mismatch! (%ld vs %ld)",
                  names_.size(),
                  joint_ranges_.size());
        exit(1);
    }
    if (joy_ranges_.size() != names_.size()) {
        ROS_ERROR("Joint name count and joy range count mismatch! (%ld vs %ld)",
                  names_.size(),
                  joy_ranges_.size());
        exit(1);
    }
}

const sensor_msgs::JointState Remapper::remap(const sensor_msgs::Joy::ConstPtr& joy) const
{
    const size_t n_axes = joy->axes.size();
    if (n_axes < n_channels_) {
        ROS_WARN_ONCE("There are less controller axes than defined joint states! (%ld vs %ld)",
                      n_axes,
                      n_channels_);
    }
    const size_t n_joint_states = std::min(n_axes, n_channels_);

    sensor_msgs::JointState joint_state;
    joint_state.name = std::vector<std::string>(names_.cbegin(), names_.cbegin() + n_joint_states);
    joint_state.position.resize(n_joint_states);

    for (size_t i = 0; i < n_joint_states; ++i) {
        joint_state.position[i] = remapRange(joy->axes[i], joy_ranges_[i], joint_ranges_[i]);
    }

    joint_state.header.stamp = ros::Time::now();
    return joint_state;
}

}  // namespace voodoo
