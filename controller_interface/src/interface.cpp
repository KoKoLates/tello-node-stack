
#include <math.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

// Services for refernce pose setup
#include "controller_interface/set_reference.h"

namespace interface {
    // references
    static double x_ref = 0.0;
    static double y_ref = 0.0;
    static double z_ref = 0.0;
    static double yaw_ref = 0.0;

    // current status
    static double x_cur = 0.0;
    static double y_cur = 0.0;
    static double z_cur = 0.0;

    // control effort
    static double u_x_cur = 0.0;
    static double u_y_cur = 0.0;
    static double u_z_cur = 0.0;
    static double u_yaw_cur = 0.0;

    static double q_x_cur = 0.0;
    static double q_y_cur = 0.0;
    static double q_z_cur = 0.0;
    static double q_w_cur = 1.0;

    static double roll_cur = 0.0;
    static double pitch_cur = 0.0;
    static double yaw_cur = 0.0;
    static ros::Time time_stamp;

    // The flag to trigger the controller
    bool REFERENCE_IS_NOT_SET = true;
}
using namespace interface;

double quadternion2yaw(double qx, double qy, double qz, double qw) {
    return atan2((qw * qz + qx * qy) * 2.0, 1 - (qy * qy + qz * qz) * 2.0);
}

void stateCallback(const geometry_msgs::PoseStamped &currentPose) {
    time_stamp = currentPose.header.stamp;

    x_cur = currentPose.pose.position.x;
    y_cur = currentPose.pose.position.y;
    z_cur = currentPose.pose.position.z;

    q_x_cur = currentPose.pose.orientation.x;
    q_y_cur = currentPose.pose.orientation.y;
    q_z_cur = currentPose.pose.orientation.z;
    q_w_cur = currentPose.pose.orientation.w;
}

void TFStateCallback(const tf2_msgs::TFMessage currentPose) {
    time_stamp = currentPose.transforms[0].header.stamp;
    x_cur = currentPose.transforms[0].transform.translation.x;
    y_cur = currentPose.transforms[0].transform.translation.y;
    z_cur = currentPose.transforms[0].transform.translation.z;

    q_x_cur = currentPose.transforms[0].transform.rotation.x;
    q_y_cur = currentPose.transforms[0].transform.rotation.y;
    q_z_cur = currentPose.transforms[0].transform.rotation.z;
    q_w_cur = currentPose.transforms[0].transform.rotation.w;

}

bool set_reference_pose(controller_interface::set_reference::Request &request, 
                        controller_interface::set_reference::Response &response) {
    x_ref = request.x_ref;
    y_ref = request.y_ref;
    z_ref = request.z_ref;
    yaw_ref = request.yaw_ref;
    ROS_INFO("[INFO] Setup the reference pose: (x: %lf, y: %lf, z: %lf, yaw: %lf)", 
                x_ref, y_ref, z_ref, yaw_ref);
    response.update_status = true;
    REFERENCE_IS_NOT_SET = false;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_interface");
    ros::NodeHandle control_interface_node;

    while(ros::ok() && ros::Time(0) == ros::Time::now()) {
        ROS_INFO("Control interface is waiting for time becom non-zero state.");
        sleep(1);
    }
    ROS_INFO("Control interface node start!");

    // Initialize the publisher for advertise
    ros::Publisher x_publisher = control_interface_node.advertise<std_msgs::Float64>("/error_x", 1);
    ros::Publisher y_publisher = control_interface_node.advertise<std_msgs::Float64>("/error_y", 1);
    ros::Publisher z_publisher = control_interface_node.advertise<std_msgs::Float64>("/error_z", 1);
    ros::Publisher yaw_publisher = control_interface_node.advertise<std_msgs::Float64>("/error_yaw", 1);
    ros::Publisher zero_setpoint_publisher = control_interface_node.advertise<std_msgs::Float64>("/zero_setpoint", 1);

    std_msgs::Float64 error_x, error_y, error_z, error_yaw, zero_setpoint;
    zero_setpoint.data = 0.0;

    // Initialize the subscriber
    ros::Subscriber state_subscriber = control_interface_node.subscribe("/orb_slam/pose", 1, stateCallback);
    ros::Subscriber ekf_state_subscriber = control_interface_node.subscribe("/tf", 1, TFStateCallback);

    // Initialize the advertise service to update reference position
    ros::ServiceServer set_reference_server = control_interface_node.advertiseService("/set_reference", set_reference_pose);

    int loop_counter = 0;
    ros::Rate loop_rate(100.0); // loop rate in 100 Hz

    double error_x_ref = 0.0, error_y_ref = 0.0;
    while(REFERENCE_IS_NOT_SET) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()) {
        ROS_INFO("Current Pose: (x: %.03lf, y: %.03lf, z: %.03lf, yaw: %.03lf", x_cur, y_cur, z_cur, yaw_cur);
        yaw_cur = quadternion2yaw(q_x_cur, q_y_cur, q_z_cur, q_w_cur);

        error_x_ref = x_ref - x_cur;
        error_y_ref = y_ref - y_cur;

        error_x.data = error_x_ref * cos(yaw_cur) + error_y_ref * sin(yaw_cur);
        error_y.data = error_x_ref * sin(yaw_cur) - error_y_ref * cos(yaw_cur);
        error_z.data = z_ref - z_cur;
        error_yaw.data = yaw_ref - yaw_cur;

        // tolerance error
        if(abs(error_x.data) < 0.02) 
            error_x.data = 0.0;
        if(abs(error_y.data) < 0.02) 
            error_y.data = 0.0;
        if(abs(error_z.data) < 0.02) 
            error_z.data = 0.0;
        if(abs(error_yaw.data) < 0.1) 
            error_yaw.data = 0.0;


        // Publish the advertise to the PID controller node
        x_publisher.publish(error_x);
        y_publisher.publish(error_y);
        z_publisher.publish(error_z);
        yaw_publisher.publish(error_yaw);
        ROS_INFO("Pose Error: (x: %.03lf, y: %.03lf, z: %.03lf, yaw: %.03lf", 
                    error_x_ref, error_y_ref, error_z.data, error_yaw.data);

        // publish the zero setpoints
        zero_setpoint_publisher.publish(zero_setpoint);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


