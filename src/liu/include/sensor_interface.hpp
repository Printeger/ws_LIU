/**
 * @file sensor_interface.h
 * @brief 定义传感器接口抽象基类
 * @author XNG
 */

#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "multi_sensor_fusion/sensor_data.hpp"
#include "nlink_parser/LinktrackNodeframe3.h"
#include "nlink_parser/LinktrackTagframe0.h"

typedef nlink_parser::LinktrackNodeframe3 UWBType;

namespace sensor_fusion {
enum SensorMask : uint8_t {
  UWB_BIT = 0b00000001,     // 0：UWB
  IMU_BIT = 0b00000010,     // 1：IMU
  LIDAR_BIT = 0b00000100,   // 2：LiDAR
  CAMERA_BIT = 0b00001000,  // 3：Camera
};

enum OdometryMode {
  UO = 0,     // UWB Odometry
  IUO = 1,    // UWB Inertial Odometry
  LIUO = 2,   // LiDAR Inertial UWB Odometry
  LIO = 3,    // LiDAR Inertial Odometry
  LICUO = 4,  // LiDAR Inertial UWB Camera Odometry
};

struct SensorConfig {
  struct UWB {
    bool is_available = false;
    std::string uwb_range_topic;
    int num_uwb = 0;
    std::vector<std::string> AnchorId;
    std::vector<double> AnchorPos;
    std::vector<double> antennaOffset;
  } uwb;

  struct IMU {
    bool is_available = false;
    std::string imu_topic;
    double imu_excite_threshold = 0.0;
    double gravity_mag = 9.81;
    bool if_normalized = false;
    bool if_use_init_bg = false;
    std::vector<double> IMUExtrinsics_Trans;
    std::vector<double> IMUExtrinsics_Rot;
  } imu;

  struct LiDAR {
    bool is_available = false;
    std::string topic;
    bool is_livox = false;
    std::vector<double> Extrinsics_Trans;
    std::vector<double> Extrinsics_Rot;
    int num_lidars = 0;
    bool lidar_timestamp_end = false;
    // if use livox
    int n_scans = 0;
    double blind = 0.0;
    double group_size = 0.0;
    double point_filter_num = 0.0;
    double inf_bound = 0.0;
    double disA = 0.0;
    double disB = 0.0;
    double limit_maxmid = 0.0;
    double limit_midmin = 0.0;
    double limit_maxmin = 0.0;
    double p2l_ratio = 0.0;
    double jump_up_limit = 0.0;
    double jump_down_limit = 0.0;
    double edgea = 0.0;
    double edgeb = 0.0;
    double smallp_intersect = 0.0;
    double smallp_ratio = 0.0;
    double edge_threshold = 0.0;
    double surf_threshold = 0.0;
    double odometry_surface_leaf_size = 0.0;
    double min_distance = 0.0;
    double max_distance = 0.0;
    bool use_corner_feature = false;
    // current_scan_param:
    double corner_leaf_size = 0.0;
    double surface_leaf_size = 0.0;
    double edge_min_valid_num = 0.0;
    double surf_min_valid_num = 0.0;
    double correspondence_downsample = 0.0;
    // keyframe_strategy:
    double angle_degree = 0.0;
    double dist_meter = 0.0;
    double time_second = 0.0;
    // map_param:
    double keyframe_search_radius = 0.0;
    double keyframe_search_time = 0.0;
    double keyframe_density = 0.0;
    double cloud_reserved_time = 0.0;  // [s]
  } lidar;

  struct Camera {
    bool is_available = false;
    bool if_compressed = false;
    std::string image_topic;
    int image_width = 0;
    int image_height = 0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    std::vector<double> cam_d;  // d0-d4
    std::vector<double> CameraExtrinsics_Trans;
    std::vector<double> CameraExtrinsics_Rot;
  } camera;
};

/**
 * @brief 传感器类型枚举
 */
enum class SensorType { IMU, LIDAR, CAMERA, GPS, UWB, ODOMETRY, UNKNOWN };

enum LiDARType {
  VLP = 0,
  LIVOX,
};

struct NextMsgs {
  NextMsgs()
      : scan_num(0),
        lidar_timestamp(-1),
        lidar_max_timestamp(-1),
        lidar_raw_cloud(new RTPointCloud),
        lidar_surf_cloud(new RTPointCloud),
        lidar_corner_cloud(new RTPointCloud),
        if_have_image(false),
        image_timestamp(-1),
        image(cv::Mat()),
        if_have_uwb(false),
        uwb_msg(),
        uwb_timestamp(-1),
        uwb_position(Eigen::Vector3d::Zero()) {}

  void Clear() {
    scan_num = 0;
    lidar_timestamp = -1;
    lidar_max_timestamp = -1;
    lidar_raw_cloud->clear();
    lidar_surf_cloud->clear();
    lidar_corner_cloud->clear();

    // image_feature_msgs.clear();
  }

  void CheckData() {
    double max_time[3];
    max_time[0] = pcl::GetCloudMaxTimeNs(lidar_surf_cloud) * NS_TO_S;
    max_time[1] = pcl::GetCloudMaxTimeNs(lidar_corner_cloud) * NS_TO_S;
    max_time[2] = pcl::GetCloudMaxTimeNs(lidar_raw_cloud) * NS_TO_S;
    LOG(INFO) << "[surf | corn | raw | max] " << max_time[0] << " "
              << max_time[1] << " " << max_time[2] << " "
              << lidar_max_timestamp * NS_TO_S;
    for (int i = 0; i < 3; i++) {
      if ((max_time[i] - lidar_max_timestamp * NS_TO_S) > 1e-6)
        std::cout << YELLOW << "[CheckData] Problem !! " << i
                  << " desired max time: " << lidar_max_timestamp * NS_TO_S
                  << "; computed max_time: " << max_time[i] << "\n"
                  << RESET;
    }

    if (image.empty()) {
      // std::cout << "[CheckData current scan img empty]\n";
    }
  }

  int scan_num = 0;
  int64_t lidar_timestamp;      // w.r.t. the start time of the trajectory
  int64_t lidar_max_timestamp;  // w.r.t. the start time of the trajectory
  RTPointCloud::Ptr lidar_raw_cloud;
  RTPointCloud::Ptr lidar_surf_cloud;
  RTPointCloud::Ptr lidar_corner_cloud;

  bool if_have_image;       // if has image in current time interval
  int64_t image_timestamp;  // w.r.t. the start time of the trajectory
  cv::Mat image;            // raw image

  bool if_have_uwb;
  int64_t uwb_timestamp;
  Eigen::Vector3d uwb_position;
  // nlink_parser::LinktrackTagframe0 uwb_msg;
  UwbData uwb_msg;
};

/**
 * @brief 传感器接口抽象基类
 *
 * 所有具体传感器类必须实现此接口
 */
template <typename T>
class ISensor {
 public:
  virtual void registerMsgCallback(const std::function<void(const T&)>& cb) = 0;

  virtual ~ISensor() = default;

  /**
   * @brief 初始化传感器
   * @param config YAML配置节点
   * @return 是否初始化成功
   */
  virtual bool initialize(const YAML::Node& config) = 0;

  /**
   * @brief 启动传感器数据接收
   * @return 是否成功启动
   */
  virtual bool start() = 0;

  /**
   * @brief 停止传感器数据接收
   * @return 是否成功停止
   */
  virtual bool stop() = 0;

  /**
   * @brief 获取传感器名称
   * @return 传感器名称
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 获取传感器类型
   * @return 传感器类型枚举
   */
  virtual SensorType getType() const = 0;

  /**
   * @brief 检查传感器是否有效
   * @return 传感器是否有效
   */
  virtual bool isValid() const = 0;

  /**
   * @brief 获取最新传感器数据
   * @return 最新的传感器数据
   */
  virtual SensorData getLatestData() const = 0;
};

/**
 * @brief 传感器工厂类
 *
 * 负责创建具体的传感器实例
 */
class SensorFactory {
 public:
  static std::shared_ptr<ISensor> create(const std::string& type,
                                         const YAML::Node& config) {
    if (type == "IMU") return std::make_shared<ImuSensor>(config);
    if (type == "LiDAR") return std::make_shared<LidarSensor>(config);
    if (type == "Camera") return std::make_shared<CameraSensor>(config);
    if (type == "GPS") return std::make_shared<GpsSensor>(config);
    if (type == "UWB") return std::make_shared<UwbSensor>(config);  // 预留
    throw std::runtime_error("Unknown sensor type");
  }
};

class UwbSensor : public ISensor {
 private:
  std::string topic_;
  ros::Subscriber sub_;
  int64_t timestamp;
  int64_t anchor_num;
  int64_t tag_num;
  std::unordered_map<int, Eigen::Vector3d> anchor_positions;
  std::unordered_map<int, double> anchor_distances;
  Eigen::Vector3d tag_position;
  bool is_time_wrt_traj_start;
  float fp_rssi;  // 第一路径信号强度
  float rx_rssi;  // 总接收信号强度
  // “rx_rssi - fp_rssi”小于6dB 时，很有可能处于视距（LOS）状态；
  // 当大于10dB时，很有可能处于非视距（NLOS）或多径状态，

  void uwbCallback(const UWBType::ConstPtr msg) {}

  /**
   * @brief 加载传感器配置
   * @param config YAML配置节点
   */
  void loadConfig(const YAML::Node& config);

  std::string name_;  ///< 传感器名称
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      imu_sub_;                    ///< IMU订阅器
  SensorData latest_data_;         ///< 最新传感器数据
  mutable std::mutex data_mutex_;  ///< 数据互斥锁
  bool initialized_ = false;       ///< 初始化状态标志
  int64_t last_timestamp_ = 0;     ///< 上次数据时间戳

 public:
  explicit ImuSensor(const rclcpp::NodeOptions& options);
  ~ImuSensor() override;
  bool initialize(const YAML::Node& config) override;
  bool start() override;
  bool stop() override;
  std::string getName() const override { return name_; }
  SensorType getType() const override { return SensorType::UWB; }
  bool isValid() const override;
};

}  // namespace sensor_fusion