//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "avp_map.h"
#include "ros_viewer.h"
#include "planner.h"

bool newGoalPoint = false;
Eigen::Vector2d goal(0, 0);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("Received 2D Nav Goal:");
    ROS_INFO("Position: [x: %.2f, y: %.2f, z: %.2f]",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Orientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
             msg->pose.orientation.x, msg->pose.orientation.y,
             msg->pose.orientation.z, msg->pose.orientation.w);

    newGoalPoint = true;
    goal.x() = msg->pose.position.x;
    goal.y() = msg->pose.position.y;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "avp_planning_node");
    ros::NodeHandle nh("~");
    ros::Subscriber sub1 = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);

    AvpMap avp_map;
    avp_map.load(DATASET_PATH "avp_map.bin");
    RosViewer rosViewer(nh);
    // visualize avp map
    rosViewer.displayAvpMap(avp_map, 0);

    // visualize vehicle
    TimedPose startPose;
    startPose.time_ = 0;
    startPose.t_= Eigen::Vector3d(19, 16, 0);
    startPose.R_ = Eigen::Quaterniond(1, 0, 0, 0);
    rosViewer.publishPose(startPose);

    // get obstacles
    std::vector<Eigen::Vector3d> obstaclePoints;
    for (const auto &p : avp_map.getSemanticElement((SemanticLabel::kSlot))) {
        Eigen::Vector3d tmp_p(p.x() * kPixelScale,
                p.y() * kPixelScale,
                p.z() * kPixelScale);
        obstaclePoints.push_back(tmp_p);
    }

    // convert start/end/obstaclePts in 2D grid coordinate with resolution 0.5m*0.5m
    double resolution = 0.5;
    Vector2i start_index(round(startPose.t_(0) / resolution),
                   round(startPose.t_(1) / resolution));

    unordered_set<Vector2i, Vector2iHash> obstacles_index;
    for (auto& p : obstaclePoints) {
        Vector2i obs_p(round(p.x() / resolution),
                     round(p.y() / resolution));
        obstacles_index.insert(obs_p);
    }

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        if(!newGoalPoint)
            continue;

        newGoalPoint = false;
        Vector2i goal_index(round(goal.x() / resolution),
                      round(goal.y() / resolution));

        // find path
        Planner planner;
        // BFS
        vector<Vector2i> path = planner.Bfs(start_index, goal_index, obstacles_index);
        // publish result
        if (path.empty()) {
            ROS_WARN("Fail to find BFS path");
        } else {
            std::vector<Eigen::Vector2d> path_points;
            for (auto &index : path) {
                Eigen::Vector2d tmp (index.x() * resolution, index.y() * resolution);
                path_points.emplace_back(tmp);
            }
            rosViewer.publishTrajectory(path_points);
        }

        // A*
        vector<Vector2i> pathA = planner.AStar(start_index, goal_index, obstacles_index);
        // publish result
        if (path.empty()) {
            ROS_WARN("Fail to find A* path");
        } else {
            std::vector<Eigen::Vector2d> path_points;
            for (auto &index : pathA) {
                Eigen::Vector2d tmp (index.x() * resolution, index.y() * resolution);
                path_points.emplace_back(tmp);
            }
            rosViewer.publishATrajectory(path_points);
        }

        // 车辆参数  for hybrid A*
        double wheelbase = 2.5;   // 轴距
        double step_size = 0.5;   // 步长
        double max_steer = M_PI / 4; // 最大转向角（45度）
        State startState(startPose.t_(0), startPose.t_(1), M_PI / 2, 0, 0, nullptr);
        State goalState(goal.x(), goal.y(), 0, 0, 0, nullptr);
        std::vector<State*> hybridApath = planner.HybridAStar(startState, goalState,
                                                               obstacles_index,
                                                                wheelbase, step_size, max_steer);
        // publish result
        if (hybridApath.empty()) {
            ROS_WARN("Fail to find Hybrid A* path");
        } else {
            std::vector<Eigen::Vector2d> path_points;
            for (auto &pState : hybridApath) {
                Eigen::Vector2d tmp (pState->x, pState->y);
                path_points.emplace_back(tmp);
            }
            rosViewer.publishHybridATrajectory(path_points);
        }

    }

    ros::spin();
}