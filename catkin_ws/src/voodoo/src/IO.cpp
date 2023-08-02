#include "voodoo/IO.h"

#include <ros/ros.h>

#include <string>
#include <tuple>
#include <vector>

#include "voodoo/voodoo.h"

namespace voodoo::IO {

const remap_param_t loadParams(const ros::NodeHandle& node_handle, const std::string& param_path)
{
    const std::string path_to_names        = param_path + "joint_names";
    const std::string path_to_joint_ranges = param_path + "joint_ranges";
    const std::string path_to_joy_ranges   = param_path + "joy_ranges";

    std::vector<std::string> names;
    if (!node_handle.getParam(path_to_names, names)) {
        ROS_ERROR("Couldn't read joint names from param %s", path_to_names.c_str());
        exit(1);
    };

    XmlRpc::XmlRpcValue joint_ranges_raw;
    if (!node_handle.getParam(path_to_joint_ranges, joint_ranges_raw)) {
        ROS_ERROR("Couldn't read joint ranges from param %s", path_to_joint_ranges.c_str());
        exit(1);
    };

    XmlRpc::XmlRpcValue joy_ranges_raw;
    if (!node_handle.getParam(path_to_joy_ranges, joy_ranges_raw)) {
        ROS_ERROR("Couldn't read joy ranges from param %s", path_to_joy_ranges.c_str());
        exit(1);
    };

    std::vector<Range<double>> joint_ranges;

    for (int i = 0; i < joint_ranges_raw.size(); ++i) {
        XmlRpc::XmlRpcValue pair = joint_ranges_raw[0];
        double minimum           = pair[0];
        double maximum           = pair[1];
        joint_ranges.emplace_back(minimum, maximum);
    }

    std::vector<Range<double>> joy_ranges;
    for (int i = 0; i < joy_ranges_raw.size(); ++i) {
        XmlRpc::XmlRpcValue pair = joy_ranges_raw[0];
        double minimum           = pair[0];
        double maximum           = pair[1];
        joy_ranges.emplace_back(minimum, maximum);
    }

    return {names, joint_ranges, joy_ranges};
}

}  // namespace voodoo::IO