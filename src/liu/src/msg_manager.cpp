#include <../include/msg_manager.h>
#include <pcl-1.13/pcl/common/transforms.h>
#include <utils/parameter_struct.h>

using namespace sensor_fusion;

namespace sensor_fusion {
MsgManager::MsgManager(const YAML::Node &node, ros::NodeHandle &nh)
    : has_valid_msg_(true),
      t_offset_imu_(0),
      t_offset_camera_(0),
      cur_imu_timestamp_(-1),
      cur_pose_timestamp_(-1),
      use_image_(false),
      lidar_timestamp_end_(false),
      remove_wrong_time_imu_(false),
      if_normalized_(false),
      image_topic_(""),
      uwb_topic_(""),
      anchor_id_positions() {
  // OdometryMode odom_mode = OdometryMode(node["odometry_mode"].as<int>());
  auto mask_ = LoadConfig(node);
  OdometryMode odom_mode = selectOdometry(mask_);
  processMask(mask_, nh);
  // LoadBag(node);
}

OdometryMode MsgManager::selectOdometry(uint8_t sensor_mask) {
  // 预定义的决策规则（按优先级从高到低）
  switch (sensor_mask) {
    case (IMU_BIT | LIDAR_BIT | CAMERA_BIT | UWB_BIT):
      return LICUO;  // 0b00001111
    case (IMU_BIT | LIDAR_BIT | UWB_BIT):
      return LIUO;  // 0b00001110
    case (IMU_BIT | LIDAR_BIT):
      return LIO;  // 0b00000110
    case (IMU_BIT | UWB_BIT):
      return IUO;  // 0b00001001
    case (IMU_BIT):
      return UO;  // 0b00000001
    default:
      throw std::runtime_error(
          "No valid odometry type for this sensor combination!");
  }
}

uint8_t MsgManager::LoadConfig(const YAML::Node &node) {
  sensor_mask = 0;
  if (node["sensors"]["UWB"]["is_available"].as<int>() == 1) {
    ROS_INFO("UWB available. ");
    sensor_mask |= UWB_BIT;
    config_.uwb.is_available = true;
    config_.uwb.uwb_range_topic =
        node["sensors"]["UWB"]["uwb_range_topic"].as<std::string>();
    config_.uwb.num_uwb = node["sensors"]["UWB"]["num_uwb"].as<int>();
    std::vector<int> anchor_ids =
        node["sensors"]["UWB"]["AnchorId"].as<std::vector<int>>();
    std::vector<double> anchor_positions =
        node["sensors"]["UWB"]["AnchorPos"].as<std::vector<double>>();

    for (size_t i = 0; i < anchor_ids.size(); i++) {
      Eigen::Vector3d pos(anchor_positions[i * 3], anchor_positions[i * 3 + 1],
                          anchor_positions[i * 3 + 2]);
      anchor_id_positions[anchor_ids[i]] = pos;
    }
  }
  if (node["sensors"]["IMU"]["is_available"].as<int>() == 1) {
    ROS_INFO("IMU available. ");
    sensor_mask |= IMU_BIT;
    config_.imu.is_available = true;
    config_.imu.imu_topic =
        node["sensors"]["IMU"]["imu_topic"].as<std::string>();
    config_.imu.imu_excite_threshold =
        node["sensors"]["IMU"]["imu_excite_threshold"].as<double>();
    config_.imu.gravity_mag =
        node["sensors"]["IMU"]["gravity_mag"].as<double>();
    config_.imu.if_normalized =
        node["sensors"]["IMU"]["if_normalized"].as<bool>();
  }

  return sensor_mask;
}

// void MsgManager::LogInfo() const {
//   int m_size[3] = {0, 0, 0};
//   m_size[0] = imu_buf_.size();
//   m_size[1] = lidar_buf_.size();
//   // if (use_image_) m_size[2] = feature_tracker_node_->NumImageMsg();
//   LOG(INFO) << "imu/lidar/image msg left: " << m_size[0] << "/" << m_size[1]
//             << "/" << m_size[2];
// }

// void MsgManager::RemoveBeginData(int64_t start_time,             // not used
//                                  int64_t relative_start_time) {  // 0
//   for (auto iter = lidar_buf_.begin(); iter != lidar_buf_.end();) {
//     if (iter->timestamp < relative_start_time) {
//       if (iter->max_timestamp <= relative_start_time) {  // [1]
//         iter = lidar_buf_.erase(iter);
//         continue;
//       } else {  // [2]
//         // int64_t t_aft = relative_start_time + 1e-3;  //1e-3
//         int64_t t_aft = relative_start_time;
//         LiDARCloudData scan_bef, scan_aft;
//         scan_aft.timestamp = t_aft;
//         scan_aft.max_timestamp = iter->max_timestamp;
//         pcl::FilterCloudByTimestamp(iter->raw_cloud, t_aft,
//         scan_bef.raw_cloud,
//                                     scan_aft.raw_cloud);
//         pcl::FilterCloudByTimestamp(iter->surf_cloud, t_aft,
//                                     scan_bef.surf_cloud,
//                                     scan_aft.surf_cloud);
//         pcl::FilterCloudByTimestamp(iter->corner_cloud, t_aft,
//                                     scan_bef.corner_cloud,
//                                     scan_aft.corner_cloud);

//         iter->timestamp = t_aft;
//         *iter->raw_cloud = *scan_aft.raw_cloud;
//         *iter->surf_cloud = *scan_aft.surf_cloud;
//         *iter->corner_cloud = *scan_aft.corner_cloud;
//       }
//     }

//     iter++;
//   }

//   if (use_image_) {
//     for (auto iter = image_buf_.begin(); iter != image_buf_.end();) {
//       if (iter->timestamp < relative_start_time) {
//         iter = image_buf_.erase(iter);  //
//         continue;
//       }
//       iter++;
//     }
//   }
//   for (auto iter = uwb_buf_.begin(); iter != uwb_buf_.end();) {
//     if (iter->timestamp < relative_start_time) {
//       iter = uwb_buf_.erase(iter);  //
//       LOG(INFO) << "Remove begin uwb data";
//       continue;
//     }
//     iter++;
//   }
// }

// bool MsgManager::HasEnvMsg() const {
//   int env_msg = lidar_buf_.size();
//   if (cur_imu_timestamp_ < 0 && env_msg > 100)
//     LOG(WARNING) << "No IMU data. CHECK imu topic" << imu_topic_;

//   return env_msg > 0;
// }

// bool MsgManager::CheckMsgIsReady(double traj_max, double start_time,
//                                  double knot_dt, bool in_scan_unit) const {
//   double t_imu_wrt_start = cur_imu_timestamp_ - start_time;

//   //
//   if (t_imu_wrt_start < traj_max) {
//     return false;
//   }

//   //
//   int64_t t_front_lidar = -1;
//   // Count how many unique lidar streams
//   std::vector<int> unique_lidar_ids;
//   for (const auto &data : lidar_buf_) {
//     if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
//                   data.lidar_id) != unique_lidar_ids.end())
//       continue;
//     unique_lidar_ids.push_back(data.lidar_id);

//     //
//     t_front_lidar = std::max(t_front_lidar, data.max_timestamp);
//   }

//   //
//   if ((int)unique_lidar_ids.size() != num_lidars_) return false;

//   //
//   int64_t t_back_lidar = lidar_max_timestamps_[0];
//   for (auto t : lidar_max_timestamps_) {
//     t_back_lidar = std::min(t_back_lidar, t);
//   }

//   //
//   if (in_scan_unit) {
//     //
//     if (t_front_lidar > t_imu_wrt_start) return false;
//   } else {
//     //
//     if (t_back_lidar < traj_max) return false;
//   }

//   return true;
// }

// bool MsgManager::AddImageToMsg(NextMsgs &msgs, const ImageData &image,
//                                int64_t traj_max) {
//   if (image.timestamp >= traj_max) return false;
//   msgs.if_have_image = true;  // important!
//   msgs.image_timestamp = image.timestamp;
//   msgs.image = image.image;
//   // msgs.image = image.image.clone();
//   return true;
// }

// bool MsgManager::AddToMsg(NextMsgs &msgs,
//                           std::deque<LiDARCloudData>::iterator scan,
//                           int64_t traj_max) {
//   bool add_entire_scan = false;
//   // if (scan->timestamp > traj_max) return add_entire_scan;

//   if (scan->max_timestamp < traj_max) {  //
//     *msgs.lidar_raw_cloud += (*scan->raw_cloud);
//     *msgs.lidar_surf_cloud += (*scan->surf_cloud);
//     *msgs.lidar_corner_cloud += (*scan->corner_cloud);

//     //
//     if (msgs.scan_num == 0) {
//       // first scan
//       msgs.lidar_timestamp = scan->timestamp;
//       msgs.lidar_max_timestamp = scan->max_timestamp;
//     } else {
//       msgs.lidar_timestamp = std::min(msgs.lidar_timestamp, scan->timestamp);
//       msgs.lidar_max_timestamp =
//           std::max(msgs.lidar_max_timestamp, scan->max_timestamp);
//     }

//     add_entire_scan = true;
//   } else {  //
//     LiDARCloudData scan_bef, scan_aft;
//     pcl::FilterCloudByTimestamp(scan->raw_cloud, traj_max,
//     scan_bef.raw_cloud,
//                                 scan_aft.raw_cloud);
//     pcl::FilterCloudByTimestamp(scan->surf_cloud, traj_max,
//     scan_bef.surf_cloud,
//                                 scan_aft.surf_cloud);
//     pcl::FilterCloudByTimestamp(scan->corner_cloud, traj_max,
//                                 scan_bef.corner_cloud,
//                                 scan_aft.corner_cloud);
//     //
//     scan_bef.timestamp = scan->timestamp;
//     scan_bef.max_timestamp = traj_max - 1e-9 * S_TO_NS;
//     scan_aft.timestamp = traj_max;
//     scan_aft.max_timestamp = scan->max_timestamp;

//     //
//     scan->timestamp = traj_max;
//     // *scan.max_timestamp = ； //
//     *scan->raw_cloud = *scan_aft.raw_cloud;
//     *scan->surf_cloud = *scan_aft.surf_cloud;
//     *scan->corner_cloud = *scan_aft.corner_cloud;

//     *msgs.lidar_raw_cloud += (*scan_bef.raw_cloud);
//     *msgs.lidar_surf_cloud += (*scan_bef.surf_cloud);
//     *msgs.lidar_corner_cloud += (*scan_bef.corner_cloud);

//     //
//     if (msgs.scan_num == 0) {
//       // first scan
//       msgs.lidar_timestamp = scan_bef.timestamp;
//       msgs.lidar_max_timestamp = scan_bef.max_timestamp;
//     } else {
//       msgs.lidar_timestamp = std::min(msgs.lidar_timestamp,
//       scan_bef.timestamp); msgs.lidar_max_timestamp =
//           std::max(msgs.lidar_max_timestamp, scan_bef.max_timestamp);
//     }

//     add_entire_scan = false;
//   }

//   //
//   msgs.scan_num++;

//   return add_entire_scan;
// }

///
// bool MsgManager::GetMsgs(NextMsgs &msgs, int64_t traj_last_max,
//                          int64_t traj_max, int64_t start_time) {
//   msgs.Clear();

//   if (imu_buf_.empty() || lidar_buf_.empty()) {
//     return false;
//   }
//   if (cur_imu_timestamp_ - start_time < traj_max) {
//     return false;
//   }

//   /// 1
//   //
//   std::vector<int> unique_lidar_ids;
//   for (const auto &data : lidar_buf_) {
//     if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
//                   data.lidar_id) != unique_lidar_ids.end())
//       continue;
//     unique_lidar_ids.push_back(data.lidar_id);
//   }
//   if (unique_lidar_ids.size() != num_lidars_) {
//     return false;
//   }
//   //
//   for (auto t : lidar_max_timestamps_) {
//     if (t < traj_max) {
//       return false;
//     }
//   }
//   //
//   if (use_image_) {
//     if (image_max_timestamp_ < traj_max) {
//       return false;
//     }
//   }

//   /// 2
//   for (auto it = lidar_buf_.begin(); it != lidar_buf_.end();) {
//     if (it->timestamp >= traj_max) {
//       ++it;
//       continue;
//     }
//     bool add_entire_scan = AddToMsg(msgs, it, traj_max);
//     if (add_entire_scan) {
//       it = lidar_buf_.erase(it);  //
//     } else {
//       ++it;  //
//     }
//   }
//   LOG(INFO) << "[msgs_scan_num] " << msgs.scan_num;

//   /// 3
//   if (use_image_) {
//     ///
//     int img_idx = INT_MAX;
//     for (int i = 0; i < image_buf_.size(); i++) {
//       if (image_buf_[i].timestamp >= traj_last_max &&
//           image_buf_[i].timestamp < traj_max) {
//         img_idx = i;
//       }
//       if (image_buf_[i].timestamp >= traj_max) {
//         break;
//       }
//     }

//     ///
//     // int img_idx = INT_MAX;
//     // for (int i = 0; i < image_buf_.size(); i++)
//     // {
//     //   if (image_buf_[i].timestamp >= traj_last_max &&
//     //       image_buf_[i].timestamp < traj_max)
//     //   {
//     //     img_idx = i;
//     //     break;
//     //   }
//     // }

//     if (img_idx != INT_MAX) {
//       AddImageToMsg(msgs, image_buf_[img_idx], traj_max);
//       // image_buf_.erase(image_buf_.begin() + img_idx);
//     } else {
//       msgs.if_have_image = false;
//       // std::cout << "[GetMsgs does not get a image]\n";
//       // std::getchar();
//     }
//   }

//   /// TODO 4
//   if (uwb_buf_.empty()) {
//     msgs.if_have_uwb = false;
//     LOG(INFO) << "uwb_buf_ is empty" << std::endl;
//   } else {
//     if (uwb_buf_.front().timestamp < traj_max) {
//       LOG(INFO) << "timestamp >= traj_max" << std::endl;
//       msgs.if_have_uwb = true;
//       msgs.uwb_msg = uwb_buf_.front();
//       uwb_buf_.pop_front();
//       LOG(INFO) << "uwb_buf_ pop_front: " << uwb_buf_.size();
//     } else {
//       LOG(INFO) << "timestamp < traj_max" << std::endl;
//       msgs.if_have_uwb = false;
//     }
//   }
//   return true;
// }

void MsgManager::IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg) {
  IMUData imu_data;
  IMUMsgToIMUData(imu_msg, imu_data);

  // Update current IMU timestamp
  cur_imu_timestamp_ = imu_data.timestamp;

  // Store the IMU data
  imu_buf_.push_back(imu_data);

  // Call the registered callback if any
  if (callbacks_.find(IMU) != callbacks_.end()) {
    callbacks_[IMU]();
  }
}

void MsgManager::ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg) {
  //   if (pub_img_.getNumSubscribers() != 0) {
  //     pub_img_.publish(msg);
  //   }

  //   cv_bridge::CvImagePtr cvImgPtr;
  //   cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //   if (cvImgPtr->image.empty()) {
  //     std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET
  //               << std::endl;
  //     return;
  //   }

  //   image_buf_.emplace_back();
  //   image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
  //   image_buf_.back().image = cvImgPtr->image;
  //   nerf_time_.push_back(image_buf_.back().timestamp);

  //   if (image_buf_.back().image.cols == 640 ||
  //       image_buf_.back().image.cols == 1280) {
  //     cv::resize(image_buf_.back().image, image_buf_.back().image,
  //                cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
  //   }

  //   // // for tiers
  //   // if (image_buf_.back().image.cols == 1920)
  //   // {
  //   //   cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   //   cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
  //   // }
  // }

  // void MsgManager::ImageMsgHandle(
  //     const sensor_msgs::CompressedImageConstPtr &msg) {
  //   if (pub_img_.getNumSubscribers() != 0) {
  //     cv_bridge::CvImagePtr cvImgPtr =
  //         cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //     sensor_msgs::Image imgMsg = *(cvImgPtr->toImageMsg());
  //     imgMsg.header = msg->header;  //
  //     pub_img_.publish(msg);
  //   }

  //   cv_bridge::CvImagePtr cvImgPtr;
  //   cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //   if (cvImgPtr->image.empty()) {
  //     std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET
  //               << std::endl;
  //     return;
  //   }

  //   image_buf_.emplace_back();
  //   image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
  //   image_buf_.back().image = cvImgPtr->image;
  //   nerf_time_.push_back(image_buf_.back().timestamp);

  //   // std::cout << image_buf_.back().image.rows << " " <<
  //   // image_buf_.back().image.cols << std::endl;

  //   if (image_buf_.back().image.cols == 640 ||
  //       image_buf_.back().image.cols == 1280) {
  //     cv::resize(image_buf_.back().image, image_buf_.back().image,
  //                cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
  //   }

  //   // // for mars
  //   // if (image_buf_.back().image.cols == 2448)
  //   // {
  //   //   // cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   //   cv::Size(1224, 1024), 0, 0, cv::INTER_LINEAR);
  //   //   cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   //   cv::Size(612, 512), 0, 0, cv::INTER_LINEAR);
  //   // }
  // }
}

void MsgManager::GetUWBPosInit(UwbData &measurements) {
  Eigen::Matrix<float, 5, 1> matB0;
  matB0.fill(-1);
  LOG(INFO) << "matB0: " << matB0(0, 0) << " " << matB0(1, 0) << " "
            << matB0(2, 0);

  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t1;
  R1 << 0.406165301800, -0.913799643517, 0.00000000000, 0.913799643517,
      0.406165301800, 0.000000000000, 0.000000000000, 0.000000000000,
      1.000000000000;
  t1 << -3.024960279465, 9.532732009888, -0.889378666879;
  Eigen::Isometry3d extrinsic_matrix = Eigen::Isometry3d::Identity();
  extrinsic_matrix.rotate(R1);
  extrinsic_matrix.pretranslate(t1);

  // reduce the init_position to all anchor positions
  for (auto &[anchor_id, anchor_pos] : anchor_id_positions) {
    LOG(INFO) << "old anchor_pos : " << anchor_pos.transpose();

    Eigen::Vector4d anchor_pos_4d =
        Eigen::Vector4d(anchor_pos.x(), anchor_pos.y(), anchor_pos.z(), 1);
    Eigen::Vector4d anchor_pos_4d_transformed =
        extrinsic_matrix * anchor_pos_4d;
    anchor_pos = Eigen::Vector3d(anchor_pos_4d_transformed.x(),
                                 anchor_pos_4d_transformed.y(),
                                 anchor_pos_4d_transformed.z());
    LOG(INFO) << "anchor_id: " << anchor_id
              << " | new anchor_pos: " << anchor_pos.transpose();
    anchor_id_positions[anchor_id] = anchor_pos;
  }
}

void MsgManager::GetUWBPos(UwbData &measurements) {
  if (measurements.anchor_distances.size() >= 3) {  // 至少需要3个锚点
    Eigen::Vector3d uwb_position = Eigen::Vector3d::Zero();

    const size_t n = anchor_id_positions.size();
    Eigen::MatrixXd A(n - 1, 3);
    Eigen::VectorXd b(n - 1);

    // 获取第一个锚点作为参考
    auto it = anchor_id_positions.begin();
    const Eigen::Vector3d &anchor0 = it->second;
    const double distance0 = measurements.anchor_distances.at(it->first);
    ++it;

    // 构建线性方程组
    for (size_t i = 0; i < n - 1; ++i, ++it) {
      const Eigen::Vector3d &anchor = it->second;
      const double distance = measurements.anchor_distances.at(it->first);

      A(i, 0) = 2 * (anchor.x() - anchor0.x());
      A(i, 1) = 2 * (anchor.y() - anchor0.y());
      A(i, 2) = 2 * (anchor.z() - anchor0.z());

      const double d0_sq = distance0 * distance0;
      const double di_sq = distance * distance;
      const double anchor0_sq = anchor0.squaredNorm();
      const double anchor_i_sq = anchor.squaredNorm();

      b(i) = d0_sq - di_sq - anchor0_sq + anchor_i_sq;
    }

    // 求解最小二乘解: x = (A^T A)^-1 A^T b
    uwb_position = A.householderQr().solve(b);

    // save the uwb_position in txt
    std::ofstream outfile(
        "/home/mint/ws_uav_setup/src/Coco-LIC-UWB/data/uwb_position.txt",
        std::ios::app);
    if (outfile.is_open()) {
      outfile << std::setprecision(19) << measurements.timestamp * 1e-9 << " "
              << uwb_position[0] << " " << uwb_position[1] << " " << 0 << " "
              << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
      outfile.close();
    } else {
      std::cerr << "Unable to open file uwb_position.txt" << std::endl;
    }
  } else {
    LOG(INFO) << YELLOW
              << "\n⚠️  Insufficient UWB measurements for initialization: "
              << measurements.anchor_distances.size() << "/3" << RESET;
  }
}

void MsgManager::CalcExtrinsic() {
  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t1;
  R1 << 0.406165301800, -0.913799643517, 0.00000000000, 0.913799643517,
      0.406165301800, 0.000000000000, 0.000000000000, 0.000000000000,
      1.000000000000;
  t1 << -3.024960279465, 9.532732009888, -0.889378666879;
  Eigen::Isometry3d extrinsic_matrix = Eigen::Isometry3d::Identity();
  extrinsic_matrix.rotate(R1);
  extrinsic_matrix.pretranslate(t1);
}

void MsgManager::UwbMsgHandle(
    const nlink_parser::LinktrackNodeframe3::ConstPtr &uwb_msg) {
  UwbData temp_uwb_data;
  temp_uwb_data.timestamp = uwb_msg->header.stamp.nsec;
  LOG(INFO) << "[Debug] UWB timestamp: " << temp_uwb_data.timestamp;

  int anchor_num = 0;
  for (auto node_ : uwb_msg->nodes) {
    temp_uwb_data.anchor_distances.emplace(node_.id, node_.dis);
    temp_uwb_data.fp_rssi = node_.fp_rssi;
    temp_uwb_data.rx_rssi = node_.rx_rssi;
    anchor_num++;
  }

  LOG(INFO) << "UWB anchor_num: " << anchor_num;
  temp_uwb_data.anchor_num = anchor_num;

  // GetUWBPos(temp_uwb_data);
  if (!is_uwb_init_) {
    GetUWBPosInit(temp_uwb_data);
    is_uwb_init_ = true;
  }
  temp_uwb_data.anchor_positions = anchor_id_positions;

  uwb_buf_.emplace_back(temp_uwb_data);
}

}  // namespace sensor_fusion
