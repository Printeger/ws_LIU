#include <glog/logging.h>
#include <ros/package.h>
#include <ros/ros.h>

// #include "include/FaultDetector.h"
// #include "include/KalmanFilter.h"
// #include "include/TimeSynchronizer.h"
#include "include/msg_manager.h"
#include "include/sensor_interface.hpp"

using namespace sensor_fusion;

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

  MsgManager msg_manager(config_node, nh);
  msg_manager.processMask(msg_manager.sensor_mask, nh);

  // if ((sensor_mask >> UWB_INDEX) & 0x01) {
  //   // 构造或获取UWB消息
  //   auto uwb_msg = ...;
  //   msg_manager.UwbMsgHandle(uwb_msg);
  // }

  // // 初始化核心模块
  // TimeSynchronizer time_sync(nh);
  // KalmanFilter fusion_core;
  // FaultDetector fault_monitor;

  // // 建立数据链路
  // for (auto* sensor : sensors) {
  //   sensor->registerCallback(
  //       [&](const SensorData& data) { time_sync.addData(data); });
  // }

  // time_sync.registerCallback([&](const SyncedData& synced_data) {
  //   if (fault_monitor.checkData(synced_data)) {
  //     fusion_core.process(synced_data);
  //   }
  // });

  // // 设置输出接口
  // auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("fused_pose", 10);
  // fusion_core.setOutputCallback([&](const FusionResult& result) {
  //   pose_pub.publish(result.toPoseMsg());
  // });

  // 启动系统
  // ROS_INFO("Sensor fusion system initialized");
  // ros::AsyncSpinner spinner(4);  // 使用4线程处理
  // spinner.start();
  // ros::waitForShutdown();

  return 0;
}