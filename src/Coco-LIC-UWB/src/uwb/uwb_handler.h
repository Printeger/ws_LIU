#pragma once

#include <nlink_parser/LinktrackTagframe0.h>
#include <ros/ros.h>
#include <spline/trajectory.h>
#include <utils/cloud_tool.h>
#include <utils/yaml_utils.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <string>
#include <vector>
namespace cocolic {
// 😃
class UWBHandler {
 public:
  typedef std::shared_ptr<UWBHandler> Ptr;
  UWBHandler(const YAML::Node &node);
  ~UWBHandler() {};
  // Process incoming UWB measurements
  void ParseUWBData(const nlink_parser::LinktrackTagframe0::ConstPtr &uwb_msg);

  // Publish processed UWB data
  void PublishUWBData();

  // Save UWB data to file
  void SaveUWBData();

  // Getters
  bool HasNewMeasurement() const { return has_new_measurement_; }
  const nlink_parser::LinktrackTagframe0 &GetLatestMeasurement() const {
    return latest_measurement_;
  }

 private:
  // ROS related
  ros::NodeHandle nh_;
  ros::Publisher pub_uwb_data_;
  ros::Subscriber sub_uwb_data_;

  // UWB data storage
  std::vector<nlink_parser::LinktrackTagframe0> uwb_data_;
  nlink_parser::LinktrackTagframe0 latest_measurement_;

  // Configuration
  std::string uwb_data_path_;
  std::string uwb_data_file_;
  std::string uwb_data_topic_;
  int uwb_data_num_;

  // State flags
  bool is_uwb_data_saved_;
  bool has_new_measurement_;
};
}  // namespace cocolic
