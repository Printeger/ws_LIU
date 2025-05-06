#include <../include/msg_manager.h>
#include <pcl-1.13/pcl/common/transforms.h>
#include <utils/parameter_struct.h>

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

  LoadBag(node);
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
  uint8_t mask = 0;
  if (node["sensors"]["UWB"]["is_available"].as<int>() == 1) {
    ROS_INFO("UWB available. ");
    mask |= UWB_BIT;
    config_.uwb.is_available = true;
    config_.uwb.uwb_range_topic =
        node["sensors"]["UWB"]["uwb_range_topic"].as<std::string>();
    config_.uwb.num_uwb = node["sensors"]["UWB"]["num_uwb"].as<int>();
    std::vector<int> anchor_ids = node["AnchorId"].as<std::vector<int>>();
    std::vector<double> anchor_positions =
        node["AnchorPos"].as<std::vector<double>>();

    for (size_t i = 0; i < anchor_ids.size() - 1;
         i++) {  // -1是因为最后一个是移动模块
      Eigen::Vector3d pos(anchor_positions[i * 3], anchor_positions[i * 3 + 1],
                          anchor_positions[i * 3 + 2]);
      anchor_id_positions[anchor_ids[i]] = pos;
    }
  }
  if (node["sensors"]["IMU"]["is_available"].as<int>() == 1) {
    ROS_INFO("IMU available. ");
    mask |= IMU_BIT;
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
  if (node["sensors"]["LiDAR"]["is_available"].as<int>() == 1) {
    ROS_INFO("LiDAR available. ");
    mask |= LIDAR_BIT;
    bool use_livox = false;
    bool use_vlp = false;
    config_.lidar.is_available = true;
    config_.lidar.topic = node["sensors"]["LiDAR"]["topic"].as<std::string>();
    config_.lidar.is_livox = node["sensors"]["LiDAR"]["is_livox"].as<bool>();
    config_.lidar.num_lidars = node["sensors"]["LiDAR"]["num_lidars"].as<int>();
    config_.lidar.lidar_timestamp_end =
        node["sensors"]["LiDAR"]["lidar_timestamp_end"].as<bool>();
    if (config_.lidar.is_livox) {
      config_.lidar.n_scans = node["sensors"]["LiDAR"]["n_scans"].as<int>();
      config_.lidar.blind = node["sensors"]["LiDAR"]["blind"].as<double>();
      config_.lidar.group_size =
          node["sensors"]["LiDAR"]["group_size"].as<double>();
      config_.lidar.point_filter_num =
          node["sensors"]["LiDAR"]["point_filter_num"].as<double>();
      config_.lidar.inf_bound =
          node["sensors"]["LiDAR"]["inf_bound"].as<double>();
      config_.lidar.disA = node["sensors"]["LiDAR"]["disA"].as<double>();
      config_.lidar.disB = node["sensors"]["LiDAR"]["disB"].as<double>();
      config_.lidar.limit_maxmid =
          node["sensors"]["LiDAR"]["limit_maxmid"].as<double>();
      config_.lidar.limit_midmin =
          node["sensors"]["LiDAR"]["limit_midmin"].as<double>();
      config_.lidar.limit_maxmin =
          node["sensors"]["LiDAR"]["limit_maxmin"].as<double>();
      config_.lidar.p2l_ratio =
          node["sensors"]["LiDAR"]["p2l_ratio"].as<double>();
      config_.lidar.jump_up_limit =
          node["sensors"]["LiDAR"]["jump_up_limit"].as<double>();
      config_.lidar.jump_down_limit =
          node["sensors"]["LiDAR"]["jump_down_limit"].as<double>();
      config_.lidar.edgea = node["sensors"]["LiDAR"]["edgea"].as<double>();
      config_.lidar.edgeb = node["sensors"]["LiDAR"]["edgeb"].as<double>();
      config_.lidar.smallp_intersect =
          node["sensors"]["LiDAR"]["smallp_intersect"].as<double>();
      config_.lidar.smallp_ratio =
          node["sensors"]["LiDAR"]["smallp_ratio"].as<double>();
      config_.lidar.edge_threshold =
          node["sensors"]["LiDAR"]["edge_threshold"].as<double>();
      config_.lidar.surf_threshold =
          node["sensors"]["LiDAR"]["surf_threshold"].as<double>();
      config_.lidar.odometry_surface_leaf_size =
          node["sensors"]["LiDAR"]["odometry_surface_leaf_size"].as<double>();
      config_.lidar.min_distance =
          node["sensors"]["LiDAR"]["min_distance"].as<double>();
      config_.lidar.max_distance =
          node["sensors"]["LiDAR"]["max_distance"].as<double>();

      lidar_types.push_back(LIVOX);
      use_livox = true;
    } else {
      lidar_types.push_back(VLP);
      use_vlp = true;
    }
    config_.lidar.use_corner_feature =
        node["sensors"]["LiDAR"]["use_corner_feature"].as<bool>();
    config_.lidar.corner_leaf_size =
        node["sensors"]["LiDAR"]["current_scan_param"]["corner_leaf_size"]
            .as<double>();
    config_.lidar.surface_leaf_size =
        node["sensors"]["LiDAR"]["current_scan_param"]["surface_leaf_size"]
            .as<double>();
    config_.lidar.edge_min_valid_num =
        node["sensors"]["LiDAR"]["current_scan_param"]["edge_min_valid_num"]
            .as<double>();
    config_.lidar.surf_min_valid_num =
        node["sensors"]["LiDAR"]["current_scan_param"]["surf_min_valid_num"]
            .as<double>();
    config_.lidar.correspondence_downsample =
        node["sensors"]["LiDAR"]["current_scan_param"]
            ["correspondence_downsample"]
                .as<double>();
    config_.lidar.angle_degree =
        node["sensors"]["LiDAR"]["keyframe_strategy"]["angle_degree"]
            .as<double>();
    config_.lidar.dist_meter =
        node["sensors"]["LiDAR"]["keyframe_strategy"]["dist_meter"]
            .as<double>();
    config_.lidar.time_second =
        node["sensors"]["LiDAR"]["keyframe_strategy"]["time_second"]
            .as<double>();
    config_.lidar.keyframe_search_radius =
        node["sensors"]["LiDAR"]["map_param"]["keyframe_search_radius"]
            .as<double>();
    config_.lidar.keyframe_search_time =
        node["sensors"]["LiDAR"]["map_param"]["keyframe_search_time"]
            .as<double>();
    config_.lidar.keyframe_density =
        node["sensors"]["LiDAR"]["map_param"]["keyframe_density"].as<double>();
    config_.lidar.cloud_reserved_time =
        node["sensors"]["LiDAR"]["map_param"]["cloud_reserved_time"]
            .as<double>();

    EP_LktoI_.emplace_back();
    EP_LktoI_.back().Init(node["Extrinsics"]["Extrinsics"]);
    for (int k = 0; k < num_lidars_; ++k) {
      lidar_max_timestamps_.push_back(0);
      Eigen::Matrix4d T_Lk_to_L0 = Eigen::Matrix4d::Identity();
      if (k > 0) {
        T_Lk_to_L0.block<3, 3>(0, 0) =
            (EP_LktoI_[0].q.inverse() * EP_LktoI_[k].q).toRotationMatrix();
        T_Lk_to_L0.block<3, 1>(0, 3) =
            EP_LktoI_[0].q.inverse() * (EP_LktoI_[k].p - EP_LktoI_[0].p);
      }
      T_LktoL0_vec_.push_back(T_Lk_to_L0);
    }
    if (use_livox)
      livox_feature_extraction_ =
          std::make_shared<LivoxFeatureExtraction>(lidar_node);
    if (use_vlp)
      velodyne_feature_extraction_ =
          std::make_shared<VelodyneFeatureExtraction>(lidar_node);
  }
  if (node["sensors"]["Camera"]["is_available"].as<int>() == 1) {
    ROS_INFO("Camera available. ");
    mask |= CAMERA_BIT;
    config_.camera.is_available = true;
    config_.camera.image_topic =
        node["sensors"]["Camera"]["image_topic"].as<std::string>();
    config_.camera.if_compressed =
        node["sensors"]["Camera"]["if_compressed"].as<bool>();
  }
  return mask;
}

void MsgManager::LoadBag(const YAML::Node &node) {
  double bag_start = node["bag_start"].as<double>();
  double bag_durr = node["bag_durr"].as<double>();

  std::vector<std::string> topics;
  topics.push_back(imu_topic_);  // imu
  if (use_image_)                // camera
    topics.push_back(image_topic_);
  for (auto &v : lidar_topics_)  // lidar
    topics.push_back(v);
  // topics.push_back(pose_topic_);
  topics.push_back(uwb_topic_);

  bag_.open(bag_path_, rosbag::bagmode::Read);

  rosbag::View view_full;
  view_full.addQuery(bag_);
  ros::Time time_start = view_full.getBeginTime();
  time_start += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                         : time_start + ros::Duration(bag_durr);
  view_.addQuery(bag_, rosbag::TopicQuery(topics), time_start, time_finish);
  if (view_.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return;
  }

  std::cout << "\n🍺 LoadBag " << bag_path_ << " start at " << bag_start
            << " with duration " << (time_finish - time_start).toSec() << ".\n";
  LOG(INFO) << "LoadBag " << bag_path_ << " start at " << bag_start
            << " with duration " << (time_finish - time_start).toSec();
}

void MsgManager::SpinBagOnce() {
  static rosbag::View::iterator view_iterator = view_.begin();
  if (view_iterator == view_.end()) {
    has_valid_msg_ = false;
    LOG(INFO) << "End of bag";
    return;
  }

  const rosbag::MessageInstance &m = *view_iterator;
  std::string msg_topic = m.getTopic();
  auto msg_time = m.getTime();

  if (msg_topic == imu_topic_)  // imu
  {
    sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    IMUMsgHandle(imu_msg);
  } else if (std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic) !=
             lidar_topics_.end())  // lidar
  {
    auto it = std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic);
    auto idx = std::distance(lidar_topics_.begin(), it);
    if (lidar_types[idx] == VLP)  //[rotating lidar: Velodyne、Ouster、Hesai]
    {
      if (!m.isType<sensor_msgs::PointCloud2>()) std::cout << "Wrong type\n";

      auto lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
      CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
      VelodyneMsgHandle(lidar_msg, idx);
      // VelodyneMsgHandleNoFeature(lidar_msg, idx);
    } else if (lidar_types[idx] == LIVOX)  //[solid-state lidar: Livox]
    {
      if (!m.isType<livox_ros_driver2::CustomMsg>())
        std::cout << "Wrong type\n";

      auto lidar_msg = m.instantiate<livox_ros_driver2::CustomMsg>();
      CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
      LivoxMsgHandle(lidar_msg, idx);  // Extract features & put in LiDAR buffer
    }
  } else if (msg_topic == image_topic_)  // camera
  {
    if (if_compressed_) {
      sensor_msgs::CompressedImageConstPtr image_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      ImageMsgHandle(image_msg);
    } else {
      sensor_msgs::ImageConstPtr image_msg =
          m.instantiate<sensor_msgs::Image>();
      ImageMsgHandle(image_msg);
    }
  } else if (msg_topic == uwb_topic_) {
    std::string uwb_topic_1 = "/nlink_linktrack_tagframe0";
    std::string uwb_topic_2 = "/nlink_linktrack_nodeframe3";
    if (uwb_topic_ == uwb_topic_1) {
      auto uwb_msg = m.instantiate<nlink_parser::LinktrackTagframe0>();
      UwbMsgHandle(uwb_msg);
      if (uwb_buf_.empty()) {
        LOG(ERROR) << "UWB buf empty ";
      }
    } else if (uwb_topic_ == uwb_topic_2) {
      auto uwb_msg = m.instantiate<nlink_parser::LinktrackNodeframe3>();
      UwbMsgHandle(uwb_msg);
      if (uwb_buf_.empty()) {
        LOG(ERROR) << "UWB buf empty ";
      }
    }
  }

  view_iterator++;
}

void MsgManager::LogInfo() const {
  int m_size[3] = {0, 0, 0};
  m_size[0] = imu_buf_.size();
  m_size[1] = lidar_buf_.size();
  // if (use_image_) m_size[2] = feature_tracker_node_->NumImageMsg();
  LOG(INFO) << "imu/lidar/image msg left: " << m_size[0] << "/" << m_size[1]
            << "/" << m_size[2];
}

void MsgManager::RemoveBeginData(int64_t start_time,             // not used
                                 int64_t relative_start_time) {  // 0
  for (auto iter = lidar_buf_.begin(); iter != lidar_buf_.end();) {
    if (iter->timestamp < relative_start_time) {
      if (iter->max_timestamp <= relative_start_time) {  // [1]
        iter = lidar_buf_.erase(iter);
        continue;
      } else {  // [2]
        // int64_t t_aft = relative_start_time + 1e-3;  //1e-3
        int64_t t_aft = relative_start_time;
        LiDARCloudData scan_bef, scan_aft;
        scan_aft.timestamp = t_aft;
        scan_aft.max_timestamp = iter->max_timestamp;
        pcl::FilterCloudByTimestamp(iter->raw_cloud, t_aft, scan_bef.raw_cloud,
                                    scan_aft.raw_cloud);
        pcl::FilterCloudByTimestamp(iter->surf_cloud, t_aft,
                                    scan_bef.surf_cloud, scan_aft.surf_cloud);
        pcl::FilterCloudByTimestamp(iter->corner_cloud, t_aft,
                                    scan_bef.corner_cloud,
                                    scan_aft.corner_cloud);

        iter->timestamp = t_aft;
        *iter->raw_cloud = *scan_aft.raw_cloud;
        *iter->surf_cloud = *scan_aft.surf_cloud;
        *iter->corner_cloud = *scan_aft.corner_cloud;
      }
    }

    iter++;
  }

  if (use_image_) {
    for (auto iter = image_buf_.begin(); iter != image_buf_.end();) {
      if (iter->timestamp < relative_start_time) {
        iter = image_buf_.erase(iter);  //
        continue;
      }
      iter++;
    }
  }
  for (auto iter = uwb_buf_.begin(); iter != uwb_buf_.end();) {
    if (iter->timestamp < relative_start_time) {
      iter = uwb_buf_.erase(iter);  //
      LOG(INFO) << "Remove begin uwb data";
      continue;
    }
    iter++;
  }
}

bool MsgManager::HasEnvMsg() const {
  int env_msg = lidar_buf_.size();
  if (cur_imu_timestamp_ < 0 && env_msg > 100)
    LOG(WARNING) << "No IMU data. CHECK imu topic" << imu_topic_;

  return env_msg > 0;
}

bool MsgManager::CheckMsgIsReady(double traj_max, double start_time,
                                 double knot_dt, bool in_scan_unit) const {
  double t_imu_wrt_start = cur_imu_timestamp_ - start_time;

  //
  if (t_imu_wrt_start < traj_max) {
    return false;
  }

  //
  int64_t t_front_lidar = -1;
  // Count how many unique lidar streams
  std::vector<int> unique_lidar_ids;
  for (const auto &data : lidar_buf_) {
    if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                  data.lidar_id) != unique_lidar_ids.end())
      continue;
    unique_lidar_ids.push_back(data.lidar_id);

    //
    t_front_lidar = std::max(t_front_lidar, data.max_timestamp);
  }

  //
  if ((int)unique_lidar_ids.size() != num_lidars_) return false;

  //
  int64_t t_back_lidar = lidar_max_timestamps_[0];
  for (auto t : lidar_max_timestamps_) {
    t_back_lidar = std::min(t_back_lidar, t);
  }

  //
  if (in_scan_unit) {
    //
    if (t_front_lidar > t_imu_wrt_start) return false;
  } else {
    //
    if (t_back_lidar < traj_max) return false;
  }

  return true;
}

bool MsgManager::AddImageToMsg(NextMsgs &msgs, const ImageData &image,
                               int64_t traj_max) {
  if (image.timestamp >= traj_max) return false;
  msgs.if_have_image = true;  // important!
  msgs.image_timestamp = image.timestamp;
  msgs.image = image.image;
  // msgs.image = image.image.clone();
  return true;
}

bool MsgManager::AddToMsg(NextMsgs &msgs,
                          std::deque<LiDARCloudData>::iterator scan,
                          int64_t traj_max) {
  bool add_entire_scan = false;
  // if (scan->timestamp > traj_max) return add_entire_scan;

  if (scan->max_timestamp < traj_max) {  //
    *msgs.lidar_raw_cloud += (*scan->raw_cloud);
    *msgs.lidar_surf_cloud += (*scan->surf_cloud);
    *msgs.lidar_corner_cloud += (*scan->corner_cloud);

    //
    if (msgs.scan_num == 0) {
      // first scan
      msgs.lidar_timestamp = scan->timestamp;
      msgs.lidar_max_timestamp = scan->max_timestamp;
    } else {
      msgs.lidar_timestamp = std::min(msgs.lidar_timestamp, scan->timestamp);
      msgs.lidar_max_timestamp =
          std::max(msgs.lidar_max_timestamp, scan->max_timestamp);
    }

    add_entire_scan = true;
  } else {  //
    LiDARCloudData scan_bef, scan_aft;
    pcl::FilterCloudByTimestamp(scan->raw_cloud, traj_max, scan_bef.raw_cloud,
                                scan_aft.raw_cloud);
    pcl::FilterCloudByTimestamp(scan->surf_cloud, traj_max, scan_bef.surf_cloud,
                                scan_aft.surf_cloud);
    pcl::FilterCloudByTimestamp(scan->corner_cloud, traj_max,
                                scan_bef.corner_cloud, scan_aft.corner_cloud);
    //
    scan_bef.timestamp = scan->timestamp;
    scan_bef.max_timestamp = traj_max - 1e-9 * S_TO_NS;
    scan_aft.timestamp = traj_max;
    scan_aft.max_timestamp = scan->max_timestamp;

    //
    scan->timestamp = traj_max;
    // *scan.max_timestamp = ； //
    *scan->raw_cloud = *scan_aft.raw_cloud;
    *scan->surf_cloud = *scan_aft.surf_cloud;
    *scan->corner_cloud = *scan_aft.corner_cloud;

    *msgs.lidar_raw_cloud += (*scan_bef.raw_cloud);
    *msgs.lidar_surf_cloud += (*scan_bef.surf_cloud);
    *msgs.lidar_corner_cloud += (*scan_bef.corner_cloud);

    //
    if (msgs.scan_num == 0) {
      // first scan
      msgs.lidar_timestamp = scan_bef.timestamp;
      msgs.lidar_max_timestamp = scan_bef.max_timestamp;
    } else {
      msgs.lidar_timestamp = std::min(msgs.lidar_timestamp, scan_bef.timestamp);
      msgs.lidar_max_timestamp =
          std::max(msgs.lidar_max_timestamp, scan_bef.max_timestamp);
    }

    add_entire_scan = false;
  }

  //
  msgs.scan_num++;

  return add_entire_scan;
}

///
bool MsgManager::GetMsgs(NextMsgs &msgs, int64_t traj_last_max,
                         int64_t traj_max, int64_t start_time) {
  msgs.Clear();

  if (imu_buf_.empty() || lidar_buf_.empty()) {
    return false;
  }
  if (cur_imu_timestamp_ - start_time < traj_max) {
    return false;
  }

  /// 1
  //
  std::vector<int> unique_lidar_ids;
  for (const auto &data : lidar_buf_) {
    if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                  data.lidar_id) != unique_lidar_ids.end())
      continue;
    unique_lidar_ids.push_back(data.lidar_id);
  }
  if (unique_lidar_ids.size() != num_lidars_) {
    return false;
  }
  //
  for (auto t : lidar_max_timestamps_) {
    if (t < traj_max) {
      return false;
    }
  }
  //
  if (use_image_) {
    if (image_max_timestamp_ < traj_max) {
      return false;
    }
  }

  /// 2
  for (auto it = lidar_buf_.begin(); it != lidar_buf_.end();) {
    if (it->timestamp >= traj_max) {
      ++it;
      continue;
    }
    bool add_entire_scan = AddToMsg(msgs, it, traj_max);
    if (add_entire_scan) {
      it = lidar_buf_.erase(it);  //
    } else {
      ++it;  //
    }
  }
  LOG(INFO) << "[msgs_scan_num] " << msgs.scan_num;

  /// 3
  if (use_image_) {
    ///
    int img_idx = INT_MAX;
    for (int i = 0; i < image_buf_.size(); i++) {
      if (image_buf_[i].timestamp >= traj_last_max &&
          image_buf_[i].timestamp < traj_max) {
        img_idx = i;
      }
      if (image_buf_[i].timestamp >= traj_max) {
        break;
      }
    }

    ///
    // int img_idx = INT_MAX;
    // for (int i = 0; i < image_buf_.size(); i++)
    // {
    //   if (image_buf_[i].timestamp >= traj_last_max &&
    //       image_buf_[i].timestamp < traj_max)
    //   {
    //     img_idx = i;
    //     break;
    //   }
    // }

    if (img_idx != INT_MAX) {
      AddImageToMsg(msgs, image_buf_[img_idx], traj_max);
      // image_buf_.erase(image_buf_.begin() + img_idx);
    } else {
      msgs.if_have_image = false;
      // std::cout << "[GetMsgs does not get a image]\n";
      // std::getchar();
    }
  }

  /// TODO 4
  if (uwb_buf_.empty()) {
    msgs.if_have_uwb = false;
    LOG(INFO) << "uwb_buf_ is empty" << std::endl;
  } else {
    if (uwb_buf_.front().timestamp < traj_max) {
      LOG(INFO) << "timestamp >= traj_max" << std::endl;
      msgs.if_have_uwb = true;
      msgs.uwb_msg = uwb_buf_.front();
      uwb_buf_.pop_front();
      LOG(INFO) << "uwb_buf_ pop_front: " << uwb_buf_.size();
    } else {
      LOG(INFO) << "timestamp < traj_max" << std::endl;
      msgs.if_have_uwb = false;
    }
  }
  return true;
}

void MsgManager::IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg) {
  int64_t t_last = cur_imu_timestamp_;
  // cur_imu_timestamp_ = imu_msg->header.stamp.toSec() -
  // add_extra_timeoffset_s_;
  cur_imu_timestamp_ = imu_msg->header.stamp.toSec() * S_TO_NS;

  IMUData data;
  IMUMsgToIMUData(imu_msg, data);

  /// problem
  // data.timestamp -= add_extra_timeoffset_s_;

  // for trajectory_manager
  imu_buf_.emplace_back(data);
}

void MsgManager::VelodyneMsgHandle(
    const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id) {
  RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
  velodyne_feature_extraction_->ParsePointCloud(vlp16_msg, vlp_raw_cloud);  //

  // transform the input cloud to Lidar0 frame
  bool is_ipnl = false;  // TODO
  if (is_ipnl) {
    Eigen::Matrix4d T_IPNL;
    T_IPNL << 0, 1, 0, 0,  //
        -1, 0, 0, 0,       //
        0, 0, 1, 0.28,     //
        0, 0, 0, 1;
    pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud, T_IPNL);
  }

  if (lidar_id != 0) {
    pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
                             T_LktoL0_vec_[lidar_id]);
  }
  //
  velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

  lidar_buf_.emplace_back();
  lidar_buf_.back().lidar_id = lidar_id;
  if (lidar_timestamp_end_) {
    lidar_buf_.back().timestamp =
        (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS;  // kaist、viral
  } else {
    lidar_buf_.back().timestamp =
        vlp16_msg->header.stamp.toSec() * S_TO_NS;  // lvi、lio
  }
  lidar_buf_.back().raw_cloud = vlp_raw_cloud;
  lidar_buf_.back().surf_cloud =
      velodyne_feature_extraction_->GetSurfaceFeature();
  lidar_buf_.back().corner_cloud =
      velodyne_feature_extraction_->GetCornerFeature();
}

void MsgManager::VelodyneMsgHandleNoFeature(
    const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id) {
  RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
  velodyne_feature_extraction_->ParsePointCloudNoFeature(vlp16_msg,
                                                         vlp_raw_cloud);  //

  // // transform the input cloud to Lidar0 frame
  // if (lidar_id != 0)
  //   pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
  //                            T_LktoL0_vec_[lidar_id]);

  // //
  // velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

  lidar_buf_.emplace_back();
  lidar_buf_.back().lidar_id = lidar_id;
  if (lidar_timestamp_end_) {
    lidar_buf_.back().timestamp =
        (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS;  // kaist、viral
  } else {
    lidar_buf_.back().timestamp =
        vlp16_msg->header.stamp.toSec() * S_TO_NS;  // lvi、lio
  }
  lidar_buf_.back().raw_cloud = vlp_raw_cloud;
  lidar_buf_.back().surf_cloud =
      velodyne_feature_extraction_->GetSurfaceFeature();
  lidar_buf_.back().corner_cloud =
      velodyne_feature_extraction_->GetCornerFeature();
}

void MsgManager::LivoxMsgHandle(
    const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg, int lidar_id) {
  RTPointCloud::Ptr livox_raw_cloud(new RTPointCloud);
  //
  // livox_feature_extraction_->ParsePointCloud(livox_msg, livox_raw_cloud);
  // livox_feature_extraction_->ParsePointCloudNoFeature(livox_msg,
  // livox_raw_cloud);
  livox_feature_extraction_->ParsePointCloudR3LIVE(livox_msg, livox_raw_cloud);

  LiDARCloudData data;
  data.lidar_id = lidar_id;
  data.timestamp = livox_msg->header.stamp.toSec() * S_TO_NS;
  data.raw_cloud = livox_raw_cloud;
  data.surf_cloud = livox_feature_extraction_->GetSurfaceFeature();
  data.corner_cloud = livox_feature_extraction_->GetCornerFeature();
  if (!data.raw_cloud->empty() && !data.surf_cloud->empty() &&
      !data.corner_cloud->empty()) {
    lidar_buf_.push_back(data);
  } else {
    ROS_WARN("Livox cloud is empty");
  }

  if (lidar_id != 0) {
    pcl::transformPointCloud(*data.raw_cloud, *data.raw_cloud,
                             T_LktoL0_vec_[lidar_id]);
    pcl::transformPointCloud(*data.surf_cloud, *data.surf_cloud,
                             T_LktoL0_vec_[lidar_id]);
    pcl::transformPointCloud(*data.corner_cloud, *data.corner_cloud,
                             T_LktoL0_vec_[lidar_id]);
  }
}

void MsgManager::ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg) {
  if (pub_img_.getNumSubscribers() != 0) {
    pub_img_.publish(msg);
  }

  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (cvImgPtr->image.empty()) {
    std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET
              << std::endl;
    return;
  }

  image_buf_.emplace_back();
  image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
  image_buf_.back().image = cvImgPtr->image;
  nerf_time_.push_back(image_buf_.back().timestamp);

  if (image_buf_.back().image.cols == 640 ||
      image_buf_.back().image.cols == 1280) {
    cv::resize(image_buf_.back().image, image_buf_.back().image,
               cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
  }

  // // for tiers
  // if (image_buf_.back().image.cols == 1920)
  // {
  //   cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
  // }
}

void MsgManager::ImageMsgHandle(
    const sensor_msgs::CompressedImageConstPtr &msg) {
  if (pub_img_.getNumSubscribers() != 0) {
    cv_bridge::CvImagePtr cvImgPtr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::Image imgMsg = *(cvImgPtr->toImageMsg());
    imgMsg.header = msg->header;  //
    pub_img_.publish(msg);
  }

  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (cvImgPtr->image.empty()) {
    std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET
              << std::endl;
    return;
  }

  image_buf_.emplace_back();
  image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
  image_buf_.back().image = cvImgPtr->image;
  nerf_time_.push_back(image_buf_.back().timestamp);

  // std::cout << image_buf_.back().image.rows << " " <<
  // image_buf_.back().image.cols << std::endl;

  if (image_buf_.back().image.cols == 640 ||
      image_buf_.back().image.cols == 1280) {
    cv::resize(image_buf_.back().image, image_buf_.back().image,
               cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
  }

  // // for mars
  // if (image_buf_.back().image.cols == 2448)
  // {
  //   // cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   cv::Size(1224, 1024), 0, 0, cv::INTER_LINEAR);
  //   cv::resize(image_buf_.back().image, image_buf_.back().image,
  //   cv::Size(612, 512), 0, 0, cv::INTER_LINEAR);
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
    const nlink_parser::LinktrackTagframe0::ConstPtr &uwb_msg) {
  UwbData temp_uwb_data;
  temp_uwb_data.timestamp = uwb_msg->local_time * 1e3;
  // LOG(INFO) << "UWB timestamp: " << temp_uwb_data.timestamp;

  int anchor_num = 0;
  for (size_t i = 0; i < uwb_msg->dis_arr.size(); i++) {
    if (uwb_msg->dis_arr[i] > 1e-3) {
      temp_uwb_data.anchor_distances[i] = uwb_msg->dis_arr[i];
      anchor_num++;
    } else {
      LOG(INFO) << "Jump UWB anchor " << i
                << " distance: " << uwb_msg->dis_arr[i];
      continue;
    }
  }
  LOG(INFO) << "UWB anchor_num: " << anchor_num;
  temp_uwb_data.anchor_num = anchor_num;
  temp_uwb_data.tag_position = Eigen::Vector3d(
      uwb_msg->pos_3d[0], uwb_msg->pos_3d[1], uwb_msg->pos_3d[2]);

  // GetUWBPos(temp_uwb_data);
  if (!is_uwb_init_) {
    GetUWBPosInit(temp_uwb_data);
    is_uwb_init_ = true;
  }
  temp_uwb_data.anchor_positions = anchor_id_positions;

  uwb_buf_.emplace_back(temp_uwb_data);
}

}  // namespace sensor_fusion
