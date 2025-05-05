#include <glog/logging.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "include/FaultDetector.h"
#include "include/KalmanFilter.h"
#include "include/TimeSynchronizer.h"
#include "include/sensor_interface.hpp"

int main(int argc, char** argv) {
  // 初始化系统
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "sensor_fusion_node");
  ros::NodeHandle nh("~");

  // 加载参数配置
  std::vector<std::string> sensor_types;
  nh.getParam("sensor_types", sensor_types);
  const std::string config_path = nh.param<std::string>("config_path", "");
  ROS_INFO("Odometry load %s.", config_path.c_str());
  YAML::Node config_node = YAML::LoadFile(config_path);

  // 创建传感器实例
  std::vector<SensorDriver*> sensors;
  for (const auto& type : sensor_types) {
    if (auto* sensor = SensorFactory::createSensor(type)) {
      sensor->initialize(config_file);
      sensors.push_back(sensor);
      LOG(INFO) << "Initialized sensor: " << type;
    }
  }

  // 初始化核心模块
  TimeSynchronizer time_sync(nh);
  KalmanFilter fusion_core;
  FaultDetector fault_monitor;

  // 建立数据链路
  for (auto* sensor : sensors) {
    sensor->registerCallback(
        [&](const SensorData& data) { time_sync.addData(data); });
  }

  time_sync.registerCallback([&](const SyncedData& synced_data) {
    if (fault_monitor.checkData(synced_data)) {
      fusion_core.process(synced_data);
    }
  });

  // 设置输出接口
  auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("fused_pose", 10);
  fusion_core.setOutputCallback([&](const FusionResult& result) {
    pose_pub.publish(result.toPoseMsg());
  });

  // 启动系统
  ROS_INFO("Sensor fusion system initialized");
  ros::AsyncSpinner spinner(4);  // 使用4线程处理
  spinner.start();
  ros::waitForShutdown();

  // 清理资源
  for (auto* sensor : sensors) delete sensor;
  return 0;
}