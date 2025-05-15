#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "avp_mapping/avp_mapping.h"

geometry_msgs::Point to_ros(const Eigen::Vector3d &pt) {
  geometry_msgs::Point ret;
  ret.x = pt.x();
  ret.y = pt.y();
  ret.z = pt.z();
  return ret;
}

class RosViewer : public ViewerInterface {
 public:
  RosViewer(ros::NodeHandle &n, bool imshow, bool autorun) : nh_(n), ViewerInterface(imshow, autorun) {
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_global_dash_pts_ = n.advertise<sensor_msgs::PointCloud>("global_dash_pts", 1000);
    pub_global_arrow_pts_ = n.advertise<sensor_msgs::PointCloud>("global_arrow_pts", 1000);
    pub_global_slot_pts_ = n.advertise<sensor_msgs::PointCloud>("global_slot_pts", 1000);
    pub_mesh_ = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    pub_slot_marker_ = n.advertise<visualization_msgs::Marker>("slot_vector", 100, true);
    pub_slot_marker_ipm_ = n.advertise<visualization_msgs::Marker>("slot_vector_ipm", 100, true);
    path_.header.frame_id = "world";
  }
  ~RosViewer(){
    ros::shutdown();
  }

  void displayAvpMap(const TimedPose &pose, const Map &avp_map, const std::vector<Slot> &cur_slots) override {
    // publish pose
    publishPose(pose);
    // publish slot
    publishSlots(avp_map.getAllSlots(), pose.time_, pub_slot_marker_, true);
    publishSlots(cur_slots, pose.time_, pub_slot_marker_ipm_, false);
    // publish semantic points
    auto grid_arrow = avp_map.getSemanticElement(SemanticLabel::kArrowLine);
    publishPoints({grid_arrow.begin(), grid_arrow.end()}, pose.time_, pub_global_arrow_pts_);
    auto grid_dash = avp_map.getSemanticElement(SemanticLabel::kDashLine);
    publishPoints({grid_dash.begin(), grid_dash.end()}, pose.time_, pub_global_dash_pts_);

    ros::spinOnce();
  }

 private:
  void publishSlots(const std::vector<Slot> &slots, double time, ros::Publisher &publisher, bool avp_slots){
    if (slots.empty()) {
      return;
    }
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time(time);
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.id = 0;

    if(avp_slots){
      line_list.color.g = 1.0;
    }else{
      line_list.color.r = 1.0;
    }
    line_list.color.a = 1.0;

    for(const auto &slot : slots){
        auto p0 = to_ros(slot.corners_[0]);
        auto p1 = to_ros(slot.corners_[1]);
        auto p2 = to_ros(slot.corners_[2]);
        auto p3 = to_ros(slot.corners_[3]);
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
        line_list.points.push_back(p3);
        line_list.points.push_back(p0);
    }
    publisher.publish(line_list);
  }

  void publishPoints(const std::vector<Eigen::Vector3i> &grid, double time, ros::Publisher &publisher){
    sensor_msgs::PointCloud global_cloud;
    global_cloud.header.frame_id = "world";
    global_cloud.header.stamp = ros::Time(time);
    for (Eigen::Vector3i pt: grid) {
      geometry_msgs::Point32 p;
      p.x = pt.x() * kPixelScale;
      p.y = pt.y() * kPixelScale;
      p.z = pt.z() * kPixelScale;
      global_cloud.points.push_back(p);
    }
    publisher.publish(global_cloud);
  }

  void publishPose(const TimedPose &pose){
    Eigen::Vector3d position = pose.t_;
    Eigen::Quaterniond q = pose.R_;

    // pub odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(pose.time_);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_odometry_.publish(odometry);
    // pub path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(pose.time_);
    pose_stamped.pose = odometry.pose.pose;
    path_.poses.push_back(pose_stamped);
    if (path_.poses.size() > 10000) {
      path_.poses.erase(path_.poses.begin());
    }
    pub_path_.publish(path_);
    // pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    transform.setOrigin(tf::Vector3(position(0),
                                    position(1),
                                    position(2)));
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(pose.time_),
                                          "world", "vehicle"));
    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(pose.time_);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d frontleftup2rightfrontup;
    frontleftup2rightfrontup << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * frontleftup2rightfrontup.transpose() * rot_mesh;
    Eigen::Vector3d t_mesh = q * Eigen::Vector3d(0, 1.5, 0) + position;
    meshROS.pose.orientation.w = q_mesh.w();
    meshROS.pose.orientation.x = q_mesh.x();
    meshROS.pose.orientation.y = q_mesh.y();
    meshROS.pose.orientation.z = q_mesh.z();
    meshROS.pose.position.x = t_mesh(0);
    meshROS.pose.position.y = t_mesh(1);
    meshROS.pose.position.z = t_mesh(2);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    std::string mesh_resource("package://avp_mapping/launch/car.dae");
    meshROS.mesh_resource = mesh_resource;
    pub_mesh_.publish(meshROS);
  }

  ros::NodeHandle &nh_;
  nav_msgs::Path path_; // trajectory
  ros::Publisher pub_path_, pub_odometry_, pub_mesh_;
  ros::Publisher pub_global_dash_pts_, pub_global_arrow_pts_, pub_global_slot_pts_;
  ros::Publisher pub_slot_marker_, pub_slot_marker_ipm_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping");
  ros::NodeHandle nh("~");
  bool show_img, auto_run;
  nh.param("show_img", show_img, true);
  nh.param("auto_run", auto_run, true);

  auto viewer = std::make_shared<RosViewer>(nh, show_img, auto_run);
  AvpMapping mapping;
  mapping.setViewer(viewer);

  std::string path(DATASET_PATH); // default dataset path
  if (argc > 1) {
    path = std::string(argv[1]);  // user specified dataset path
  }

  // load pose data
  std::string pose_file(path + "/poses.bin");
  std::ifstream pose_ifs(pose_file, std::ios::binary);
  if (!pose_ifs) {
    std::cerr << "Unable to open pose file: " << pose_file << std::endl;
    return 0;
  }
  std::vector<TimedPose> poses;
  io::loadVector(pose_ifs, poses);
  for (const auto &pose : poses) {
    mapping.processPose(pose);
  }
  std::cout << "poses num: " << poses.size() << std::endl;
  pose_ifs.close();

  // load and process images
  std::ifstream ipm_imgs_ifs(path + "/ipm_imgs.txt");
  if (!ipm_imgs_ifs) {
    std::cerr << "Unable to open ipm img file: " << path + "/ipm_imgs.txt" << std::endl;
    return 0;
  }
  double time;
  std::string line, img_name;
  while (ros::ok() && std::getline(ipm_imgs_ifs, line)) {
    std::stringstream ss(line);
    ss >> time >> img_name;
    std::cout << std::to_string(time) << " :  " << img_name << std::endl;
    // process image
    mapping.processImage(time, cv::imread(path + "/" + img_name));
  }

  // save map data
  mapping.getMap().save(path + "/avp_map.bin");
  return 0;
}
