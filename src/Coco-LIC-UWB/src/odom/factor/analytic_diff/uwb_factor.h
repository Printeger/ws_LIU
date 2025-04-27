#pragma once

#include <ceres/ceres.h>
#include <odom/msg_manager.h>
#include <spline/trajectory.h>
#include <utils/parameter_struct.h>

#include "split_spline_view.h"

namespace cocolic {
namespace analytic_derivative {

class UWBFactorNURBS : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  UWBFactorNURBS(int64_t time_uwb_ns,
                 const std::pair<const int, double>& dist_pair,
                 const std::pair<int, Eigen::Vector3d>& anchor_pos_pair,
                 const std::pair<int, double>& su,
                 const Eigen::Matrix4d& blending_matrix,
                 const Eigen::Matrix4d& cumulative_blending_matrix,
                 const SO3d& S_GtoM, const Vec3d& p_GinM, const SO3d& S_UtoI,
                 const Vec3d& p_UinI, double weight)
      : time_uwb_ns_(time_uwb_ns),
        dist_pair_(dist_pair),
        anchor_pos_pair_(anchor_pos_pair),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix),
        S_GtoM_(S_GtoM),
        p_GinM_(p_GinM),
        S_UtoI_(S_UtoI),
        p_UinI_(p_UinI),
        weight_(weight) {
    // LOG(INFO) << "UWB Factor Constructor";
    set_num_residuals(1);

    size_t kont_num = 4;
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // TODO: Implement UWB factor evaluation
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;
    // Extrinsic UWB to IMU
    Vec3d p_U = Vec3d::Zero();
    Vec3d p_IU = S_UtoI_ * p_U + p_UinI_;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    // Calculate rotation matrix from IMU axis to global frame, and position
    if (jacobians) {
      // LOG(INFO) << "Jacobians";
      // for (int i = 0; i < 8; ++i) {  // 8个参数块
      //   if (jacobians[i]) {
      //     const int block_size =
      //         (i < 4) ? 4 : 3;  // 前4个是姿态块(4D)，后4个是位置块(3D)
      //     Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>>(jacobians[i],
      //     1,
      //                                                          block_size)
      //         .setZero();
      //   }
      // }
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, &J_R);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, &J_p);
    } else {
      // LOG(INFO) << "!Jacobians";
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, nullptr);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, nullptr);
    }
    Vec3d p_UinM = S_GtoM_ * (S_ItoG * p_U + p_IinG) + p_GinM_;
    // LOG(INFO) << "===== UWB p_UinM: " << p_UinM.transpose()
    //           << " | S_GtoM: " << S_GtoM_.matrix()
    //           << " | p_GinM: " << p_GinM_.transpose()
    //           << " p_U: " << p_U.transpose() << " | S_ItoG: " <<
    //           S_ItoG.matrix()
    //           << " | p_IinG: " << p_IinG.transpose();
    // Calculate the position of UWB anchors in global frame
    double temp_residual = 0.0;
    // LOG(INFO) << "anchor id: " << anchor.first
    //           << " | anchor postion: " << anchor.second.transpose()
    //           << " | anchor distance: " << it->second
    //           << " | p_UinM: " << p_UinM.transpose()
    //           << " | predict distance: " << (p_UinM -
    //           anchor.second).norm();
    auto pred_dist_vec = p_UinM - anchor_pos_pair_.second;
    double pred_dist = pred_dist_vec.norm();
    double indi_resi = (dist_pair_.second - pred_dist) * weight_;
    residuals[0] = indi_resi;
    // LOG(INFO) << "anchor id: " << dist_pair.first
    //           << " | indi_resi: " << indi_resi
    //           << " | anchor distance: " << dist_pair.second
    //           << " | predict distance: " << pred_dist
    //           << " | anchor postion: " << anchor_pos_pair->second.transpose()
    //           << " | p_UinM: " << p_UinM.transpose()
    //           << " | UWB residual: " << residuals[0];
    LOG(INFO) << "===== UWB residual: " << residuals[0] << " ======";
    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < 4; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + 4]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + 4]);
          jac_kont_p.setZero();
        }
      }
    }
    Vec3d jac_lhs_R = Vec3d::Zero();
    jac_lhs_R.fill(1);
    /// Rotation control point
    for (size_t i = 0; i < 4; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 1*3 3*3
        jac_kont_R.block<1, 3>(0, 0) =
            jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < 4; i++) {
      size_t idx = 4 + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 1*3
        jac_kont_p = J_p.d_val_d_knot[i] * pred_dist_vec;
        jac_kont_p = (weight_ / pred_dist * jac_kont_p).eval();
      }
    }
    return true;
  }

 private:
  int64_t time_uwb_ns_;
  std::pair<const int, double> dist_pair_;
  std::pair<int, Eigen::Vector3d> anchor_pos_pair_;
  // UwbData uwb_data_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
  Eigen::Vector3d uwb_measurement_;
  Eigen::Matrix3d K_;
  SO3d S_UtoI_;
  Vec3d p_UinI_;
  SO3d S_GtoM_;
  Vec3d p_GinM_;
  double weight_;
};

class UWBFactorNURBSAutoDiff : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  UWBFactorNURBSAutoDiff(int64_t time_uwb_ns, const UwbData& uwb_data,
                         const std::pair<int, double>& su,
                         const Eigen::Matrix4d& blending_matrix,
                         const Eigen::Matrix4d& cumulative_blending_matrix,
                         const SO3d& S_GtoM, const Vec3d& p_GinM,
                         const SO3d& S_UtoI, const Vec3d& p_UinI, double weight)
      : time_uwb_ns_(time_uwb_ns),
        uwb_data_(uwb_data),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix),
        S_GtoM_(S_GtoM),
        p_GinM_(p_GinM),
        S_UtoI_(S_UtoI),
        p_UinI_(p_UinI),
        weight_(weight) {
    // LOG(INFO) << "UWB Factor Constructor";
    set_num_residuals(1);

    size_t kont_num = 4;
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool operator()(double const* const* parameters, double* residuals,
                          double** jacobians) const {
    // TODO: Implement UWB factor evaluation
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;
    // Extrinsic UWB to IMU
    Vec3d p_U = Vec3d::Zero();
    Vec3d p_IU = S_UtoI_ * p_U + p_UinI_;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    // Calculate rotation matrix from IMU axis to global frame, and position
    if (jacobians) {
      // LOG(INFO) << "Jacobians";
      for (int i = 0; i < 8; ++i) {  // 8个参数块
        if (jacobians[i]) {
          const int block_size =
              (i < 4) ? 4 : 3;  // 前4个是姿态块(4D)，后4个是位置块(3D)
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>>(jacobians[i], 1,
                                                               block_size)
              .setZero();
        }
      }
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, &J_R);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, &J_p);
    } else {
      // LOG(INFO) << "!Jacobians";
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, nullptr);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, nullptr);
    }
    Vec3d p_UinM = S_GtoM_ * (S_ItoG * p_U + p_IinG) + p_GinM_;
    // Calculate the position of UWB anchors in global frame
    double temp_residual = 0.0;
    for (auto anchor_dist : uwb_data_.anchor_distances) {
      auto it = uwb_data_.anchor_positions.find(anchor_dist.first);
      // LOG(INFO) << "anchor id: " << anchor.first
      //           << " | anchor postion: " << anchor.second.transpose()
      //           << " | anchor distance: " << it->second
      //           << " | p_UinM: " << p_UinM.transpose()
      //           << " | predict distance: " << (p_UinM -
      //           anchor.second).norm();
      double indi_resi =
          std::abs(anchor_dist.second - (p_UinM - it->second).norm()) * weight_;
      temp_residual += indi_resi;
      // temp_residual += 0;
      residuals[0] = temp_residual;
      // LOG(INFO) << "anchor id: " << anchor_dist.first
      //           << " | indi_resi: " << indi_resi
      //           << " | anchor distance: " << anchor_dist.second
      //           << " | predict distance: " << (p_UinM - it->second).norm()
      //           << " | anchor postion: " << it->second.transpose()
      //           << " | p_UinM: " << p_UinM.transpose()
      //           << " | UWB residual: " << residuals[0];
    }
    LOG(INFO) << "===== UWB residual: " << residuals[0] << " ======";
    return true;
  }

 private:
  int64_t time_uwb_ns_;
  UwbData uwb_data_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
  Eigen::Vector3d uwb_measurement_;
  Eigen::Matrix3d K_;
  SO3d S_UtoI_;
  Vec3d p_UinI_;
  SO3d S_GtoM_;
  Vec3d p_GinM_;
  double weight_;
};

}  // namespace analytic_derivative

// namespace auto_diff {

// }  // namespace auto_diff

}  // namespace cocolic