#pragma once

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <spdlog/spdlog.h>

#include <string>
#include <vector>

namespace voodoo {

template <typename T>
struct Range
{
    T minimum;
    T maximum;
    T span;

    Range(const T min, const T max)
        : minimum(min)
        , maximum(max)
        , span(max - min)
    {}
};

class Remapper
{
  private:
    size_t n_channels_;
    std::vector<std::string> names_;
    std::vector<Range<double>> joint_ranges_;
    std::vector<Range<double>> joy_ranges_;

  public:
    Remapper(const std::vector<std::string>& names,
             const std::vector<Range<double>>& joint_ranges,
             const std::vector<Range<double>>& joy_ranges);

    const sensor_msgs::JointState remap(const sensor_msgs::Joy::ConstPtr& joy) const;
};

}  // namespace voodoo