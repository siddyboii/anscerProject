#ifndef TRAJECTORY_PUBLISHER_SAVER_H
#define TRAJECTORY_PUBLISHER_SAVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "trajectory_visualization_storage/SaveTrajectory.h"
#include <fstream>
#include <string>

class TrajectoryPublisherSaver {
public:
    TrajectoryPublisherSaver();
    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer save_trajectory_srv_;
    std::ofstream trajectory_file_;
    geometry_msgs::PoseStamped latest_pose_;
    visualization_msgs::MarkerArray marker_array_;
    bool saveTrajectoryCallback(trajectory_visualization_storage::SaveTrajectory::Request& req, trajectory_visualization_storage::SaveTrajectory::Response& res);
    void pathCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif // TRAJECTORY_PUBLISHER_SAVER_H































// #ifndef TRAJECTORY_PUBLISHER_SAVER_H
// #define TRAJECTORY_PUBLISHER_SAVER_H

// #include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <nav_msgs/Path.h>
// #include "trajectory_visualization_storage/SaveTrajectory.h"

// class TrajectoryPublisherSaver {
// public:
//     TrajectoryPublisherSaver(); // Constructor
//     void run(); // Function to handle the main loop

// private:
//     ros::NodeHandle nh_; // NodeHandle to manage communication with ROS
//     ros::Publisher marker_pub_; // Publisher to publish MarkerArray for visualization
//     ros::Subscriber path_sub_; // Subscriber to the robot's path topic
//     ros::ServiceServer save_trajectory_srv_; // Service server to handle save requests

//     void pathCallback(const nav_msgs::Path::ConstPtr& path); // Callback for path subscription
//     bool saveTrajectory(trajectory_visualization_storage::SaveTrajectory::Request& req,
//                         trajectory_visualization_storage::SaveTrajectory::Response& res); // Service to save trajectory
//     void publishMarkers(const visualization_msgs::MarkerArray& markers); // Helper to publish markers directly
// };

// #endif // TRAJECTORY_PUBLISHER_SAVER_H

