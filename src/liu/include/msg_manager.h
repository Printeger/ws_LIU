#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
// #include <lidar/livox_feature_extraction.h>
// #include <lidar/velodyne_feature_extraction.h>
// #include <livox_ros_driver2/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/log_utils.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <map>
#include <unordered_map>
#include <utils/eigen_utils.hpp>
#include <vector>

#include "nlink_parser/LinktrackNodeframe3.h"
#include "nlink_parser/LinktrackTagframe0.h"
#include "sensor_interface.hpp"

using SensorCallback = std::function<void()>;

namespace sensor_fusion {

class MsgManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<MsgManager> Ptr;

  MsgManager(const YAML::Node &node, ros::NodeHandle &nh);

  uint8_t LoadConfig(const YAML::Node &node);

  OdometryMode selectOdometry(uint8_t sensor_mask);

  //
  void SpinBagOnce();

  //
  bool GetMsgs(NextMsgs &msgs, int64_t traj_last_max, int64_t traj_max,
               int64_t start_time);

  int64_t GetCurIMUTimestamp() const { return cur_imu_timestamp_; }

  //
  void LogInfo() const;

  int NumLiDAR() const { return num_lidars_; }

  inline void IMUMsgToIMUData(const sensor_msgs::Imu::ConstPtr &imu_msg,
                              IMUData &data) {
    data.timestamp = imu_msg->header.stamp.toSec() * S_TO_NS;
    data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                imu_msg->angular_velocity.y,
                                imu_msg->angular_velocity.z);
    if (if_normalized_) {
      data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x * 9.81,
                                   imu_msg->linear_acceleration.y * 9.81,
                                   imu_msg->linear_acceleration.z * 9.81);
    } else {
      // TODO: IPNL
      bool is_ipnl = false;
      if (is_ipnl) {
        data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.y,
                                     imu_msg->linear_acceleration.x,
                                     imu_msg->linear_acceleration.z);
      } else {
        data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                     imu_msg->linear_acceleration.y,
                                     imu_msg->linear_acceleration.z);
      }
    }
    //  imu_msg->linear_acceleration.z + 9.81);
    Eigen::Vector4d q(imu_msg->orientation.w, imu_msg->orientation.x,
                      imu_msg->orientation.y, imu_msg->orientation.z);
    if (std::fabs(q.norm() - 1) < 0.01) {
      data.orientation = SO3d(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
    }
  }

  static void CheckLidarMsgTimestamp(double ros_bag_time, double msg_time) {
    //
    double delta_time = ros_bag_time - msg_time;
    if (delta_time < 0.08) {
      LOG(INFO) << "[CheckLidarMsgTimestamp] Delta Time : " << delta_time;
    }
  }

  inline void processMask(uint8_t sensor_mask, ros::NodeHandle &nh) {
    for (int i = 0; i < 4; ++i) {
      if ((sensor_mask >> i) & 0x01) {
        auto type = static_cast<SensorType>(i);
        if (type == 0) {
          sub_uwb_ = nh.subscribe("config_.uwb_topic", 1000,
                                  &MsgManager::UwbMsgHandle, this);
        } else if (type == 1) {
          sub_imu_ = nh.subscribe("config_.imu_topic", 1000,
                                  &MsgManager::IMUMsgHandle, this);
        } else if (type == 2) {
          // for (int j = 0; j < num_lidars_; ++j) {
          //   if (lidar_types[j] == VLP) {
          //     subs_vlp16_[j] = nh.subscribe(
          //         lidar_topics_[j], 1000,
          //         boost::bind(&MsgManager::VelodyneMsgHandle, this, _1, j));
          //   } else if (lidar_types[j] == LIVOX) {
          //     subs_livox_[j] = nh.subscribe(
          //         lidar_topics_[j], 1000,
          //         boost::bind(&MsgManager::LivoxMsgHandle, this, _1, j));
          //   }
          // }
        } else if (type == 3) {
          // sub_image_ =
          //     nh.subscribe("config_.image_topic", 1000, ImageMsgHandle);
        }
      }
    }
  }

  void RemoveBeginData(int64_t start_time, int64_t relative_start_time = 0);

  void registerCallback(SensorType type, SensorCallback cb) {
    callbacks_[type] = cb;
  }

 private:
  void LoadBag(const YAML::Node &node);

  bool HasEnvMsg() const;

  //
  bool CheckMsgIsReady(double traj_max, double start_time, double knot_dt,
                       bool in_scan_unit) const;

  bool AddImageToMsg(NextMsgs &msgs, const ImageData &image, int64_t traj_max);

  //
  bool AddToMsg(NextMsgs &msgs, std::deque<LiDARCloudData>::iterator scan,
                int64_t traj_max);

  void IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg);

  void VelodyneMsgHandle(const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg,
                         int lidar_id);
  void VelodyneMsgHandleNoFeature(
      const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id);

  void LivoxMsgHandle(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg,
                      int lidar_id);

  void ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg);
  void ImageMsgHandle(const sensor_msgs::CompressedImageConstPtr &msg);

  void UwbMsgHandle(const nlink_parser::LinktrackTagframe0::ConstPtr &uwb_msg);
  void UwbMsgHandle(const nlink_parser::LinktrackNodeframe3::ConstPtr &uwb_msg);

  void GetUWBPosInit(UwbData &measurements);
  void GetUWBPos(UwbData &measurements);
  void CalcExtrinsic();

 public:
  bool has_valid_msg_;

  std::string bag_path_;

  NextMsgs cur_msgs;
  NextMsgs next_msgs;
  NextMsgs next_next_msgs;

  double t_offset_imu_;
  double t_offset_camera_;

  int64_t imu_period_ns_;
  // double add_extra_timeoffset_s_;

  double t_image_ms_;
  double t_lidar_ms_;

  std::deque<ImageData> image_buf_;
  std::vector<int64_t> nerf_time_;
  Eigen::aligned_deque<IMUData> imu_buf_;
  std::deque<LiDARCloudData> lidar_buf_;
  std::deque<UwbData> uwb_buf_;
  std::vector<int64_t> lidar_max_timestamps_;
  int64_t image_max_timestamp_;
  int64_t uwb_max_timestamp_;

  Eigen::aligned_deque<PoseData> pose_buf_;
  PoseData init_pose_;

  int64_t cur_imu_timestamp_;
  int64_t cur_pose_timestamp_;

  std::unordered_map<int, Eigen::Vector3d> anchor_id_positions;

  uint8_t sensor_mask = 0;

 private:
  // int64_t cur_imu_timestamp_;
  // int64_t cur_pose_timestamp_;

  bool use_image_;
  bool lidar_timestamp_end_;
  bool remove_wrong_time_imu_;
  bool if_normalized_;
  bool if_compressed_;
  bool is_uwb_init_ = false;

  std::string imu_topic_;
  int num_lidars_;
  std::vector<LiDARType> lidar_types;
  std::vector<std::string> lidar_topics_;
  std::string image_topic_;
  std::string uwb_topic_;
  // std::string pose_topic_;

  std::vector<ExtrinsicParam> EP_LktoI_;

  /// lidar_k 到 lidar_0 的外参
  Eigen::aligned_vector<Eigen::Matrix4d> T_LktoL0_vec_;

  rosbag::Bag bag_;
  rosbag::View view_;

  ros::Subscriber sub_uwb_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_image_;
  std::vector<ros::Subscriber> subs_vlp16_;
  std::vector<ros::Subscriber> subs_livox_;

  // VelodyneFeatureExtraction::Ptr velodyne_feature_extraction_;
  // LivoxFeatureExtraction::Ptr livox_feature_extraction_;

  ros::Publisher pub_img_;
  SensorConfig config_;

  std::unordered_map<SensorType, SensorCallback> callbacks_;
};

}  // namespace sensor_fusion
