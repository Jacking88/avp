//
// Created by ubuntu on 25-3-1.
//

#include "ros_viewer.h"


RosViewer::RosViewer(ros::NodeHandle &n){
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000, true);
    pub_reference_ = n.advertise<nav_msgs::Path>("reference", 1000, true);
    pub_pose_ = n.advertise<nav_msgs::Odometry>("pose", 1000, true);
    pub_mesh_ = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    pub_ref_pts_ = n.advertise<sensor_msgs::PointCloud>("reference_pts", 100, true);
    path_.header.frame_id = "world";
    reference_.header.frame_id = "world";
}

void RosViewer::publishReferencePoint(const double x, const double y) {
    sensor_msgs::PointCloud global_cloud;
    global_cloud.header.frame_id = "world";
    global_cloud.header.stamp = ros::Time(0);

    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = 0;
    global_cloud.points.push_back(p);
    pub_ref_pts_.publish(global_cloud);
}

void RosViewer::publishPose(const double x, const double y, const double yaw) {
    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
    Eigen::Matrix3d R;
    R << cos(yaw), -sin(yaw), 0,
    sin(yaw), cos(yaw), 0,
    0, 0, 1;
    Eigen::Quaterniond q;
    q = R;

    // pub odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(0);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_pose_.publish(odometry);
    // pub path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(0);
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
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0),
                                          "world", "vehicle"));
    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(0);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d frontleftup2rightfrontup;
    frontleftup2rightfrontup << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * rot_mesh;
    Eigen::Vector3d t_mesh = position;
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
    // std::string mesh_resource(MESH_PATH);
    std::string mesh_resource = "package://avp_control/meshes/car.dae";
    meshROS.mesh_resource = mesh_resource;
    pub_mesh_.publish(meshROS);
}

void RosViewer::publishTrajectory(const std::vector<std::vector<double>> &trajectory) {
    // pub path
    reference_.poses.clear();
    for (auto &p : trajectory) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.pose.position.x = p[0];
        pose_stamped.pose.position.y = p[1];
        pose_stamped.pose.position.z = 0;
        reference_.poses.push_back(pose_stamped);
    }
    pub_reference_.publish(reference_);
}