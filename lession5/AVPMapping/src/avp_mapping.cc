#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "avp_mapping/avp_mapping.h"

AvpMapping::AvpMapping() {
  pre_key_pose_.time_ = -1;
  T_vehicle_ipm_.linear().setIdentity();
  T_vehicle_ipm_.translation() = Eigen::Vector3d(0.0, 1.32, 0.0);
}

bool AvpMapping::isKeyFrame(const TimedPose &pose) {
  if ((pose.t_ - pre_key_pose_.t_).norm() > 0.1 ||
      std::fabs(GetYaw(pre_key_pose_.R_.inverse() * pose.R_)) > 5. * kToRad ||
      pose.time_ - pre_key_pose_.time_ > 30 || pre_key_pose_.time_ < -1) {
    pre_key_pose_ = pose;
    return true;
  }
  return false;
}

void AvpMapping::processPose(const TimedPose &pose) {
  pose_interpolation_.Push(pose);
}

void AvpMapping::processImage(double time, const cv::Mat &ipm_seg_img) {
  if (time < pose_interpolation_.EarliestTime()) {
    return;
  }
  TimedPose pose;
  pose.time_ = time;
  if (pose_interpolation_.LookUp(pose) && isKeyFrame(pose)) {
    pose_interpolation_.TrimBefore(time);
    std::cout << "keyframe : " << "  t = " << pose.t_.transpose().head(2)
              << ", yaw = " << GetYaw(pose.R_) * kToDeg << std::endl;

    cv::Mat img_gray;
    cv::cvtColor(ipm_seg_img, img_gray, cv::COLOR_BGR2GRAY);
    extractSlot(img_gray, pose);
  }
}
Eigen::Vector3d AvpMapping::ipmPlane2Global(const TimedPose &T_world_vehicle, const cv::Point2f &ipm_point){
  Eigen::Vector3d pt_global;
  //////////////////////// TODO: transform ipm pixel to point in global ///////////////////////
  // 设定 IPM 视角下的比例尺（可根据实际标定调整）
  constexpr double kPixelScale = 0.02;  // 每个像素代表的实际物理距离（单位：米）
  constexpr double kIPMOriginX = 500;   // IPM 图像的原点 X（像素）
  constexpr double kIPMOriginY = 500;   // IPM 图像的原点 Y（像素）

  // 1. IPM 视角的像素坐标 (u, v) → 车辆坐标系
  double X_vehicle = -(kIPMOriginX - ipm_point.x) * kPixelScale;
  double Y_vehicle = (kIPMOriginY - ipm_point.y) * kPixelScale;  // 像素坐标系通常是从左上角开始，Y 轴翻转

  Eigen::Vector3d pt_vehicle(X_vehicle, Y_vehicle + 1.32, 0.0);  // 设定 Z = 0，假设车位在地面

  // 2. 车辆坐标系 → 世界坐标系
  pt_global = T_world_vehicle.R_ * pt_vehicle + T_world_vehicle.t_; 

  return pt_global;
}

void AvpMapping::extractSlot(const cv::Mat &img_gray, const TimedPose &T_world_vehicle) {
  // 创建一个空的图像，用来存储库位区域
  cv::Mat slot_img = cv::Mat::zeros(img_gray.size(), img_gray.type());

  // 遍历每个像素
  for (int i = 0; i < img_gray.rows; ++i) {
      for (int j = 0; j < img_gray.cols; ++j) {
          // 根据像素值进行分类，并标记库位区域
          if (kSlotGray == img_gray.at<uchar>(i, j) || kSlotGray1 == img_gray.at<uchar>(i, j)) {
              slot_img.at<uchar>(i, j) = 254;
          } else if(kArrowGray == img_gray.at<uchar>(i, j)) {
              avp_map_.addSemanticElement(SemanticLabel::kArrowLine,
                                          {ipmPlane2Global(T_world_vehicle, cv::Point2f(j, i))});
          } else if(kDashGray == img_gray.at<uchar>(i, j)) {
              avp_map_.addSemanticElement(SemanticLabel::kDashLine,
                                          {ipmPlane2Global(T_world_vehicle, cv::Point2f(j, i))});
          }
      }
  }

  // 形态学操作：关闭操作去除噪声
  int kernelSize = 15;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(kernelSize, kernelSize));
  cv::Mat closed;
  cv::morphologyEx(slot_img, closed, cv::MORPH_CLOSE, kernel);
  auto line_img = closed.clone();

  // 骨架化，提取线条
  cv::Mat skel = skeletonize(closed);
  removeIsolatedPixels(skel, 1);

  // 使用霍夫变换检测线段
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 50, 50, 50);

  // 调用 detectSlot 来提取车位（slots）
  auto [corners, slot_points] = detectSlot(slot_img, lines);

  // 将检测到的车位加入到地图中
  std::vector<Slot> slots(slot_points.size() / 4);
  int index = 0;
  for (auto &slot : slots){
      slot.corners_[0] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
      slot.corners_[1] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
      slot.corners_[2] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
      slot.corners_[3] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
      avp_map_.addSlot(slot); // 将车位加入地图
  }

  // 更新视图
  if (viewer_) {
      viewer_->displayAvpMap(T_world_vehicle, avp_map_, slots); // 显示地图
      viewer_->displayIpmDetection(corners, slot_points, img_gray); // 显示检测到的车位
  }
}

