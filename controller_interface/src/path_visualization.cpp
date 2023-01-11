
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>

class Path_Visualizer {
    private:
        ros::NodeHandle handle;
        ros::Publisher orb_path_publisher;
        ros::Publisher ekf_path_publisher;
        ros::Publisher optiTrack_publisher;

        ros::Subscriber orb_pose_subscriber;
        ros::Subscriber ekf_pose_subscriber;
        ros::Subscriber optiTrack_subscriber;

        nav_msgs::Path orb_path, ekf_path, optiTrack_path;
        geometry_msgs::PoseStamped orb_pose, ekf_pose;

    public:
        Path_Visualizer() {
            ekf_path_publisher = handle.advertise<nav_msgs::Path>("/ekf_path", 1);
            ekf_pose_subscriber = handle.subscribe("/tf", 1, &Path_Visualizer::ekf_pose_callback, this);

            orb_path_publisher = handle.advertise<nav_msgs::Path>("/orb_path", 1);
            orb_pose_subscriber = handle.subscribe("/orb_slam/pose", 1, &Path_Visualizer::orb_pose_callback, this);
        }

        void orb_pose_callback(const geometry_msgs::PoseStamped pose) {
            if(isnan(pose.pose.position.x)) {
                ROS_WARN("[WARN] The pose is NAN.");
                return;
            }

            orb_pose.header = pose.header;
            orb_pose.pose = pose.pose;
            orb_path.header.frame_id = "map";
            orb_path.poses.push_back(orb_pose);
            orb_path_publisher.publish(orb_path);
        }

        void ekf_pose_callback(const tf2_msgs::TFMessage msg)
        {
            ekf_pose.header = msg.transforms[0].header;
            ekf_pose.pose.position.x = msg.transforms[0].transform.translation.x;
            ekf_pose.pose.position.y = msg.transforms[0].transform.translation.y;
            ekf_pose.pose.position.z = msg.transforms[0].transform.translation.z;
            ekf_pose.pose.orientation = msg.transforms[0].transform.rotation;

            ekf_path.header.frame_id = "map";
            ekf_path.poses.push_back(ekf_pose);
            ekf_path_publisher.publish(ekf_path);
        }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visualizer");
    ros::start();

    Path_Visualizer node;
    ROS_INFO("[INFO] Path visualizer node initialized.");

    ros::spin();
    ros::shutdown();
    return 0;
}