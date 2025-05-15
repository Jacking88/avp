//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

#include "ros/ros.h"
#include "ros_viewer.h"
#include "controller.h"
#include "KinematicModel.h"

vector<vector<double>> generateRefTrajectory() {
    vector<vector<double>> refer_path(1000, vector<double>(4));
    vector<double> refer_x, refer_y;
    // generate reference path
    for (int i = 0; i < 1000; i++) {
        refer_path[i][0] = 0.1 * i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }
    double dx,dy,ddx,ddy;
    for(int i=0;i<refer_path.size();i++){
        if (i==0){
            dx = refer_path[i+1][0] - refer_path[i][0];
            dy = refer_path[i+1][1] - refer_path[i][1];
            ddx = refer_path[2][0] + refer_path[0][0] - 2*refer_path[1][0];
            ddy = refer_path[2][1] + refer_path[0][1] - 2*refer_path[1][1];
        }else if(i==refer_path.size()-1){
            dx = refer_path[i][0] - refer_path[i-1][0];
            dy = refer_path[i][1] - refer_path[i-1][1];
            ddx = refer_path[i][0] + refer_path[i-2][0] - 2*refer_path[i-1][0];
            ddy = refer_path[i][1] + refer_path[i-2][1] - 2*refer_path[i-1][1];
        }else{
            dx = refer_path[i+1][0] - refer_path[i][0];
            dy = refer_path[i+1][1] - refer_path[i][1];
            ddx = refer_path[i+1][0] + refer_path[i-1][0] - 2*refer_path[i][0];
            ddy = refer_path[i+1][1] + refer_path[i-1][1] - 2*refer_path[i][1];
        }
        refer_path[i][2] = atan2(dy,dx);//yaw
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        refer_path[i][3] = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3.0 / 2); // k计算
    }
    return refer_path;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "avp_control_node");
    ros::NodeHandle nh("~");
    RosViewer rosviewer(nh);

    Controller controller;

    vector<vector<double>> refer_path = generateRefTrajectory();
    // visualize reference path
    rosviewer.publishTrajectory(refer_path);

    // Intital state [0, -1, 0.5]
    KinematicModel vehicle_model(0, -3, 0, 2, 2, 0.1);

    vector<double> x_, y_;
    vector<double> robot_state(4);
    ros::Rate loop_rate(10);
    for (int i = 0; i < 700; i++) {
        robot_state[0] = vehicle_model.x;
        robot_state[1] = vehicle_model.y;
        robot_state[2] = vehicle_model.psi;
        robot_state[3] = vehicle_model.v;

        double lateral_control;
        // lateral_control = controller.PIDController(robot_state, refer_path, vehicle_model);
        // std::cout<<"PIDController: "<<lateral_control<<std::endl;
        // lateral_control = controller.StanlyController(robot_state, refer_path, vehicle_model);
        // std::cout<<"StanlyController: "<<lateral_control<<std::endl;
        // lateral_control = controller.PurePursuitController(robot_state, refer_path, vehicle_model);
        // std::cout<<"PurePursuitController: "<<lateral_control<<std::endl;
        lateral_control = controller.LQRController(robot_state, refer_path, vehicle_model);
        std::cout<<"PurePursuitController: "<<lateral_control<<std::endl;


        // update state
        double longitudinal_control = 2.0; // constant velocity 2.0m/s
        vehicle_model.updateState(longitudinal_control, lateral_control);
        x_.push_back(vehicle_model.x);
        y_.push_back(vehicle_model.y);

        // visualize path
        rosviewer.publishPose(vehicle_model.x, vehicle_model.y, vehicle_model.psi);
        loop_rate.sleep();
    }

    ros::spin();
}