//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include "avp_localization.h"
#include "map.h"

ros::Publisher pub_estimation_path, pub_gt_path, pub_measurement_path,
    pub_odometry;
nav_msgs::Path path_estimation, path_gt, path_measurment;
ros::Publisher meshPub;

struct Node {

  Node() {
    avp_localization_.reset(new AvpLocalization(DATASET_PATH "avp_map_sim.bin"));
  }

  void addOdomGps(const nav_msgs::OdometryConstPtr &msg) {

    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    t.x() = msg->pose.pose.position.x;
    t.y() = msg->pose.pose.position.y;
    t.z() = msg->pose.pose.position.z;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;

    if (!ekf_inited_) { // set ref pose
      ekf_inited_ = true;
      // add errors to init
      avp_localization_->initState(msg->header.stamp.toSec(), t.x() - 1., t.y() + 1.,
                                   GetYaw(q) + 5. * kToRad);
    }
    // printf("gps_gt: t = %.4f, x= %.3f , y = %.3f, yaw = %.2f \n",
    //        msg->header.stamp.toSec(), t.x(), t.y(), kToDeg * GetYaw(q));
  }

  void addImuSpeed(const sensor_msgs::ImuConstPtr &imu,
                   const geometry_msgs::Vector3StampedConstPtr &speed) {
    if (ekf_inited_) {
      WheelMeasurement wheel_measurement;
      wheel_measurement.time_ = imu->header.stamp.toSec();
      wheel_measurement.velocity_ = speed->vector.y;
      wheel_measurement.yaw_rate_ = imu->angular_velocity.z;
      avp_localization_->processWheelMeasurement(wheel_measurement);
    }
  }

  void addIPmImage(const sensor_msgs::CompressedImageConstPtr &msg) {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!image.empty()) {
      double cur_time = msg->header.stamp.toSec();
      avp_localization_->processImage(cur_time, image);
    }
  }

  bool ekf_inited_{false};
  std::shared_ptr<AvpLocalization> avp_localization_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_localization_node");
  ros::NodeHandle nh("~");

  Node node;

  message_filters::Subscriber<sensor_msgs::Imu> sub0(nh, "/imu", 100);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(
      nh, "/encoder_speed", 100);

  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), sub0, sub1);
  sync.registerCallback(boost::bind(&Node::addImuSpeed, &node, _1, _2));

  ros::Subscriber gps_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom_gt", 100, boost::bind(&Node::addOdomGps, &node, _1));
  ros::Subscriber ipm_sub = nh.subscribe<sensor_msgs::CompressedImage>(
      "/ipm_seg", 10, boost::bind(&Node::addIPmImage, &node, _1));

  ros::spin();
}