#include <ros/ros.h>

#include <string>
#include <tuple>
#include <vector>

#include "voodoo/voodoo.h"

namespace voodoo::IO {

using remap_param_t =
    std::tuple<std::vector<std::string>, std::vector<Range<double>>, std::vector<Range<double>>>;

const remap_param_t loadParams(const ros::NodeHandle& node_handle, const std::string& param_path);

}  // namespace voodoo::IO