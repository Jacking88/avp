#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "avp_localization.h"
#include <pcl/common/transforms.h>

AvpLocalization::AvpLocalization(std::string map_file) {
  pre_key_pose_.time_ = -1;

  avp_map_.load(map_file);
  map_viewer_.reset(new MapViewer(avp_map_));
  T_vehicle_ipm_.linear().setIdentity();
  T_vehicle_ipm_.translation() = Eigen::Vector3d(0.0, 1.32, 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr dash_line_cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr arrow_line_cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr slot_cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto& element :
       avp_map_.getSemanticElement(SemanticLabel::kDashLine)) {
    pcl::PointXYZ pt;
    pt.x = element.x();
    pt.y = element.y();
    pt.z = 0;
    dash_line_cloud_in_->points.push_back(pt);
  }

  for (const auto& element :
       avp_map_.getSemanticElement(SemanticLabel::kArrowLine)) {
    pcl::PointXYZ pt;
    pt.x = element.x();
    pt.y = element.y();
    pt.z = 0;
    arrow_line_cloud_in_->points.push_back(pt);
  }

  for (const auto& element :
       avp_map_.getSemanticElement(SemanticLabel::kSlot)) {
    pcl::PointXYZ pt;
    pt.x = element.x();
    pt.y = element.y();
    pt.z = 0;
    slot_cloud_in_->points.push_back(pt);
  }

  kdtree_dash_line_.setInputCloud(dash_line_cloud_in_);
  kdtree_arrow_line_.setInputCloud(arrow_line_cloud_in_);
  kdtree_slot_.setInputCloud(slot_cloud_in_);
}
bool AvpLocalization::isKeyFrame(const TimedPose& pose) {
  if ((pose.t_ - pre_key_pose_.t_).norm() > 2 ||
      std::fabs(GetYaw(pre_key_pose_.R_.inverse() * pose.R_)) > 20. * kToRad ||
      pose.time_ - pre_key_pose_.time_ > 30 || pre_key_pose_.time_ < -1) {
    pre_key_pose_ = pose;
    return true;
  }
  return false;
}

void AvpLocalization::initState(double time, double x, double y, double yaw) {
  // 需要设置初始的位置！！！
  ekf_ = EKF(time, x, y, yaw);
  if (map_viewer_) {
    curr_frame_.t_update = {x, y, 0};
    map_viewer_->showFrame(curr_frame_); // init position
  }
}

void AvpLocalization::processWheelMeasurement(
    const WheelMeasurement& wheel_mearsure) {
  wheel_measurements_.push_back(wheel_mearsure);
}

// not realtime
void AvpLocalization::processImage(double time, const cv::Mat& ipm_seg_img) {
  // do predict -- for sync
  while (!wheel_measurements_.empty() &&
         wheel_measurements_.front().time_ < time) {
    ekf_.ekfPredict(wheel_measurements_.front().time_,
                    wheel_measurements_.front().velocity_,
                    wheel_measurements_.front().yaw_rate_);
    wheel_measurements_.pop_front();
  }
  if (!wheel_measurements_.empty()) {
    ekf_.ekfPredict(time, wheel_measurements_.front().velocity_,
                    wheel_measurements_.front().yaw_rate_);
  }

  if (ekf_.isInit()) {
    State state = ekf_.getState();
    TimedPose pose;
    pose.time_ = state.time;
    pose.t_ = {state.x, state.y, 0.0};
    pose.R_ = Eigen::Quaterniond(
        Eigen::AngleAxisd{state.yaw, Eigen::Vector3d::UnitZ()});

    if (!isKeyFrame(pose)) {
      return;
    }
    std::cout << std::endl;
    printf("Keyframe: time = %.4f  (%.3f, %.3f, %.2f) \n", time, state.x,
           state.y, state.yaw * kToDeg);
    cv::Mat img_gray;
    cv::cvtColor(ipm_seg_img, img_gray, cv::COLOR_BGR2GRAY);
    extractFeature(img_gray, pose);

    Eigen::Affine3d T_world_vehicle = imageRegistration();
    Eigen::Vector3d t_est = T_world_vehicle.translation();
    Eigen::Matrix3d R_est = T_world_vehicle.rotation();

    printf("ICP: time = %.4f  (%.3f, %.3f, %.2f) \n", time, t_est.x(),
           t_est.y(), GetYaw(Eigen::Quaterniond(R_est)) * kToDeg);
    ekf_.ekfUpdate(t_est.x(), t_est.y(), GetYaw(Eigen::Quaterniond(R_est)));
    std::cout << "**********************************************" << std::endl;
    if (map_viewer_) {
      state = ekf_.getState(); // updated pose
      pose.time_ = state.time;
      pose.t_ = {state.x, state.y, 0.0};
      pose.R_ = Eigen::Quaterniond(
          Eigen::AngleAxisd{state.yaw, Eigen::Vector3d::UnitZ()});

      extractFeature(img_gray, pose);
      curr_frame_.t_update = pose.t_;
      map_viewer_->showFrame(curr_frame_);
    }

  } else {
    std::cout << "Unable to get state at " << std::to_string(time) << std::endl;
  }
}

Eigen::Vector3d AvpLocalization::ipmPlane2Global(
    const TimedPose& T_world_vehicle, const cv::Point2f& ipm_point) {
  Eigen::Vector3d pt_global;
  Eigen::Affine3d T_world_ipm = Eigen::Translation3d(T_world_vehicle.t_) *
                                T_world_vehicle.R_ * T_vehicle_ipm_;
  Eigen::Vector3d pt_ipm{-(0.5 * kIPMImgHeight - ipm_point.x) * kPixelScale,
                         (0.5 * kIPMImgWidth - ipm_point.y) * kPixelScale, 0.0};
  pt_global = T_world_ipm * pt_ipm;
  return pt_global;
}

void AvpLocalization::extractFeature(const cv::Mat& img_gray,
                                     const TimedPose& T_world_vehicle) {
  curr_frame_.clearGridMap();
  curr_frame_.T_world_ipm_ = Eigen::Translation3d(T_world_vehicle.t_) *
                             T_world_vehicle.R_ * T_vehicle_ipm_;
  cv::Mat slot_img = cv::Mat::zeros(img_gray.size(), img_gray.type());
  cv::Mat seg_img = cv::Mat::zeros(img_gray.size(), img_gray.type());
  for (int i = 0; i < img_gray.rows; ++i) {
    for (int j = 0; j < img_gray.cols; ++j) {
      if (kSlotGray == img_gray.at<uchar>(i, j) ||
          kSlotGray1 == img_gray.at<uchar>(i, j)) {
        slot_img.at<uchar>(i, j) = 254;
      } else if (kArrowGray == img_gray.at<uchar>(i, j)) {
        seg_img.at<uchar>(i, j) = 254;
      } else if (kDashGray == img_gray.at<uchar>(i, j)) {
        seg_img.at<uchar>(i, j) = 254;
      }
    }
  }

  int kernelSize = 15;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(kernelSize, kernelSize));
  cv::Mat closed;
  cv::morphologyEx(slot_img, closed, cv::MORPH_CLOSE, kernel);
  auto line_img = closed.clone();

  cv::Mat skel = skeletonize(closed);
  removeIsolatedPixels(skel, 2);
  for (int i = 0; i < skel.rows; ++i) {
    for (int j = 0; j < skel.cols; ++j) {
      if (skel.at<uchar>(i, j)) {
        curr_frame_.addSemanticElement(
            SemanticLabel::kSlot,
            {ipmPlane2Global(T_world_vehicle, cv::Point2f(j, i))});
      }
    }
  }

  // find contour
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(seg_img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (const auto& contour : contours) {
    for (const auto& point : contour) {
      if (kArrowGray == img_gray.at<uchar>(point)) {
        curr_frame_.addSemanticElement(SemanticLabel::kArrowLine,
                                    {ipmPlane2Global(T_world_vehicle,
                                    point)});
      } else if (kDashGray == img_gray.at<uchar>(point)) {
        curr_frame_.addSemanticElement(SemanticLabel::kDashLine,
                                    {ipmPlane2Global(T_world_vehicle,
                                    point)});
      }
    }
  }
}

// Registration frame to map
Eigen::Affine3d AvpLocalization::imageRegistration() {
  auto slot_num = curr_frame_.getSemanticElement(SemanticLabel::kSlot).size();
  auto dash_num = slot_num + curr_frame_.getSemanticElement(SemanticLabel::kDashLine).size();
  auto total_num = dash_num + curr_frame_.getSemanticElement(SemanticLabel::kArrowLine).size();

  std::vector<char> correspondences; // 1 indicate relative point is correspondent
  pcl::PointCloud<pcl::PointXYZ> cloud_in;  // points in map
  pcl::PointCloud<pcl::PointXYZ> cloud_out, cloud_est; // points in frame

  // get points in frame
  cloud_out.reserve(total_num);
  for (const auto &grid :
         curr_frame_.getSemanticElement(SemanticLabel::kSlot)) {
    cloud_out.emplace_back(grid.x(), grid.y(), 0);
  }
  for (const auto &grid :
         curr_frame_.getSemanticElement(SemanticLabel::kDashLine)) {
    cloud_out.emplace_back(grid.x(), grid.y(), 0);
  }
  for (const auto &grid :
         curr_frame_.getSemanticElement(SemanticLabel::kArrowLine)) {
    cloud_out.emplace_back(grid.x(), grid.y(), 0);
  }
  cloud_in.resize(cloud_out.size());

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f(0, 0, 0);
  const int max_iter = 10;
  for (int n = 0; n < max_iter; n++) {
    Eigen::Affine3f T_est = Eigen::Translation3f(t) * R;
    pcl::transformPointCloud(cloud_out, cloud_est, T_est);

    /////  TODO:  finding correspondences by kd-tree nearest neighbor search ////
    correspondences.resize(total_num, 0);

    // for (size_t i = 0; i < cloud_est.size(); ++i) {
    //   int bestNode = -1;   // 记录最佳匹配点的索引
    //   float bestDist = std::numeric_limits<float>::infinity();  // 记录最佳匹配点的最小距离
    //   pcl::PointXYZ bestPoint; // 记录最佳匹配的点
  
    //   std::vector<int> pointIdxNKNSearch(1);         
    //   std::vector<float> pointNKNSquaredDistance(1); 
  
    //   // 在 dash_line KD-Tree 中查找最近邻
    //   if (kdtree_dash_line_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    //       if (pointNKNSquaredDistance[0] < bestDist && pointNKNSquaredDistance[0] < 1000) {  
    //           bestDist = pointNKNSquaredDistance[0];
    //           bestNode = pointIdxNKNSearch[0];
    //           bestPoint = dash_line_cloud_in_->points[bestNode];
    //       }
    //   }
  
    //   // 在 arrow_line KD-Tree 中查找最近邻
    //   if (kdtree_arrow_line_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    //       if (pointNKNSquaredDistance[0] < bestDist && pointNKNSquaredDistance[0] < 1000) {  
    //           bestDist = pointNKNSquaredDistance[0];
    //           bestNode = pointIdxNKNSearch[0];
    //           bestPoint = arrow_line_cloud_in_->points[bestNode];
    //       }
    //   }
  
    //   // 在 slot KD-Tree 中查找最近邻
    //   if (kdtree_slot_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    //       if (pointNKNSquaredDistance[0] < bestDist && pointNKNSquaredDistance[0] < 1000) {  
    //           bestDist = pointNKNSquaredDistance[0];
    //           bestNode = pointIdxNKNSearch[0];
    //           bestPoint = slot_cloud_in_->points[bestNode];
    //       }
    //   }
  
    //   // 如果找到合适的匹配点
    //   if (bestNode != -1) {
    //       correspondences[i] = 1;
    //       cloud_in[i] = bestPoint;
    //   }
    // }
    for (size_t i = 0; i < cloud_est.size(); ++i) {
      std::vector<int> pointIdxNKNSearch(1);         // 最近邻点索引
      std::vector<float> pointNKNSquaredDistance(1); // 最近邻点的平方距离
  
      // 在不同的 KD-Tree 中搜索最近邻点
      if (kdtree_dash_line_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
          if (pointNKNSquaredDistance[0] < 1000) { // 设定距离阈值
              correspondences[i] = 1;
              cloud_in[i] = dash_line_cloud_in_->points[pointIdxNKNSearch[0]];
          }
      } else if (kdtree_arrow_line_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
          if (pointNKNSquaredDistance[0] < 1000) {
              correspondences[i] = 1;
              cloud_in[i] = arrow_line_cloud_in_->points[pointIdxNKNSearch[0]];
          }
      } else if (kdtree_slot_.nearestKSearch(cloud_est[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
          if (pointNKNSquaredDistance[0] < 1000) { // 这个阈值可以调整
              correspondences[i] = 1;
              cloud_in[i] = slot_cloud_in_->points[pointIdxNKNSearch[0]];
          }
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////

    size_t num_pts = 0;
    Eigen::Vector3f sum_point_in = Eigen::Vector3f::Zero();
    Eigen::Vector3f sum_point_out = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < total_num; ++i) {
      if (correspondences[i]) {
        ++num_pts;
        sum_point_in += Eigen::Vector3f(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
        sum_point_out += Eigen::Vector3f(cloud_est[i].x, cloud_est[i].y, cloud_est[i].z);
      }
    }
    Eigen::Vector3f u_point_in = sum_point_in / (num_pts + 1e-10);
    Eigen::Vector3f u_point_out = sum_point_out / (num_pts + 1e-10);

    std::cout << "\ricp itr " << n + 1 << " / 10, pts: " << num_pts
          << ", dYaw: " << GetYaw(Eigen::Quaternionf(R)) * kToDeg
          << ", trans: " << t.transpose() * float(kPixelScale);
    std::cout.flush();

    ////////////  TODO: calculate close-form R t //////////////////
    // 初始化总的旋转矩阵 R_total 和总的平移向量 t_total
    static Eigen::Matrix3f R_total = Eigen::Matrix3f::Identity();
    static Eigen::Vector3f t_total = Eigen::Vector3f::Zero();

    // 记录上一次的 R 和 t
    static Eigen::Matrix3f R_prev = R_total;
    static Eigen::Vector3f t_prev = t_total;

    // 计算协方差矩阵 H
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < total_num; ++i) {
        if (correspondences[i]) { // 仅计算匹配到的点对
            Eigen::Vector3f p_in(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            Eigen::Vector3f p_out(cloud_out[i].x, cloud_out[i].y, cloud_out[i].z);

            p_in -= u_point_in;  // 去均值化
            p_out -= u_point_out;

            H += p_in * p_out.transpose();
        }
    }

    // 对 H 进行 SVD 分解
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // 计算旋转矩阵 R
    Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
    S(2, 2) = (U * V.transpose()).determinant(); // 保证旋转矩阵的正交性
    R = U * S * V.transpose();

    // 计算平移向量 t
    t = u_point_in - R * u_point_out;

    // 叠加变换
    R_total = R * R_total;  // 先乘新 R
    t_total = R * t_total + t;  // 旋转后平移

    // 检查 R 和 t 的变化是否足够小，若小于阈值则提前退出
    float R_diff = (R - R_prev).norm();  // 旋转矩阵的差异
    float t_diff = (t - t_prev).norm();  // 平移向量的差异

    if (R_diff < 1e-2 && t_diff < 1e-2) {
        std::cout << "\nConvergence reached at iteration " << n + 1 << std::endl;
        break;
    }

    // 更新上一轮的 R 和 t
    R_prev = R;
    t_prev = t;

    
  //////////////////////////////////////////////////////////////////////////
  }
  std::cout << std::endl;
  t *= float(kPixelScale); // grid scale to world scale

  Eigen::Affine3d dT =
      Eigen::Translation3d(t.cast<double>()) * R.cast<double>();
  Eigen::Affine3d T_world_vehicle =
      dT * curr_frame_.T_world_ipm_ * T_vehicle_ipm_.inverse();
  return T_world_vehicle;
}