#include "trajectory_visualization_storage/TrajectoryPublishSaver.h"
#include "ros/ros.h"

TrajectoryPublisherSaver::TrajectoryPublisherSaver()
    : nh_("~"),
      path_sub_(nh_.subscribe("/odom", 1, &TrajectoryPublisherSaver::pathCallback, this)),
      marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1)),
      save_trajectory_srv_(nh_.advertiseService("save_trajectory", &TrajectoryPublisherSaver::saveTrajectoryCallback, this)) {
    marker_array_.markers.resize(1);
    marker_array_.markers[0].header.frame_id = "map";
    marker_array_.markers[0].id = 0;
    marker_array_.markers[0].type = visualization_msgs::Marker::SPHERE;
    marker_array_.markers[0].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[0].pose = latest_pose_.pose;
    marker_array_.markers[0].scale.x = 0.1;
    marker_array_.markers[0].scale.y = 0.1;
    marker_array_.markers[0].scale.z = 0.1;
    marker_array_.markers[0].color.r = 1.0;
    marker_array_.markers[0].color.g = 0.0;
    marker_array_.markers[0].color.b = 0.0;
    marker_array_.markers[0].color.a = 1.0;

    nh_.param("duration", duration_);
    //ROS_INFO("Setting trajectory duration to %f seconds", duration_);

    start_time_ = ros::Time::now();
    ROS_INFO("Start time: %f", start_time_.toSec());


    // std::string filename = req.filename + ".csv";
    // trajectory_file_.open(filename);
    // if (!trajectory_file_.is_open()) {
    //     ROS_ERROR("Failed to open file %s for writing.", filename.c_str());
    //     res.success = false;
    //     return true;
    // }

    // trajectory_file_ << "x,y,z,qx,qy,qz,qw,timestamp" << std::endl;
}

void TrajectoryPublisherSaver::pathCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    
    if(ros::Time::now() - start_time_ > ros::Duration(duration_)){
        return;
    }


    latest_pose_.header = msg->header;
    latest_pose_.pose.position = msg->pose.pose.position;
    latest_pose_.pose.orientation = msg->pose.pose.orientation;
    marker_array_.markers[0].pose = latest_pose_.pose;
    marker_pub_.publish(marker_array_);
    if (trajectory_file_.is_open()){
        ROS_INFO("File is getting editted");
        trajectory_file_ << latest_pose_.pose.position.x << ","
                        << latest_pose_.pose.position.y << ","
                        << latest_pose_.pose.position.z << ","
                        << latest_pose_.pose.orientation.x << ","
                        << latest_pose_.pose.orientation.y << ","
                        << latest_pose_.pose.orientation.z << ","
                        << latest_pose_.pose.orientation.w << ","
                        << msg->header.stamp.toSec() << std::endl;
    }
}

bool TrajectoryPublisherSaver::saveTrajectoryCallback(trajectory_visualization_storage::SaveTrajectory::Request& req, trajectory_visualization_storage::SaveTrajectory::Response& res) {
    // if (trajectory_file_.is_open()) {
    //     trajectory_file_.close();
    // }

    std::string filename = req.filename + ".csv";
    trajectory_file_.open(filename);
    if (!trajectory_file_.is_open()) {
        ROS_ERROR("Failed to open file %s for writing.", filename.c_str());
        res.success = false;
        return false;
    }

    trajectory_file_ << "x,y,z,qx,qy,qz,qw,timestamp" << std::endl;
    res.success = true;
    ROS_INFO("File saved");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_publisher_saver");
    TrajectoryPublisherSaver trajectory_publisher_saver;
    ros::spin();
    return 0;
}














































// #include "trajectory_visualization_storage/TrajectoryPublisherSaver.h"
// #include <fstream>
// #include <sstream>
// #include <iomanip>

// TrajectoryPublisherSaver::TrajectoryPublisherSaver()
//   : nh_("~"),
//     marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1)),
//     path_sub_(nh_.subscribe("/robot/path", 1, &TrajectoryPublisherSaver::pathCallback, this)),
//     save_trajectory_srv_(nh_.advertiseService("save_trajectory", &TrajectoryPublisherSaver::saveTrajectory, this)) {
// }

// void TrajectoryPublisherSaver::run() {
//     ros::spin();
// }

// void TrajectoryPublisherSaver::pathCallback(const nav_msgs::Path::ConstPtr& path) {
//     visualization_msgs::MarkerArray markers;
//     for (size_t i = 0; i < path->poses.size(); i++) {
//         visualization_msgs::Marker marker;
//         marker.header = path->header;
//         marker.pose = path->poses[i];
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.scale.x = 0.1;
//         marker.scale.y = 0.1;
//         marker.scale.z = 0.1;
//         marker.color.r = 1.0;
//         marker.color.g = 0.0;
//         marker.color.b = 0.0;
//         marker.color.a = 1.0;
//         markers.markers.push_back(marker);
//     }
//     publishMarkers(markers);
// }

// // bool TrajectoryPublisherSaver::saveTrajectory(trajectory_visualization_storage::SaveTrajectory::Request& req,
// //                                               trajectory_visualization_storage::SaveTrajectory::Response& res) {
// //     // TODO: Implement the logic to save the trajectory to a file
// //     // For now, just return success
// //     res.success = true;
// //     return true;
// // }

// bool TrajectoryPublisherSaver::saveTrajectory(trajectory_visualization_storage::SaveTrajectory::Request& req,
//                                               trajectory_visualization_storage::SaveTrajectory::Response& res) {
//     std::string filename = req.filename;
//     double duration = req.duration;

//     // Filter the trajectory data based on the requested duration
//     std::vector<geometry_msgs::PoseStamped> filtered_trajectory;
//     double current_time = ros::Time::now().toSec();
//     for (const auto& pose : trajectory_) {
//         if (current_time - pose.header.stamp.toSec() <= duration) {
//             filtered_trajectory.push_back(pose);
//         }
//     }

//     // Save the filtered trajectory data to a CSV file
//     std::ofstream csv_file(filename);
//     if (csv_file.is_open()) {
//         csv_file << "time,x,y,z,qx,qy,qz,qw\n";
//         for (const auto& pose : filtered_trajectory) {
//             csv_file << std::fixed << std::setprecision(6) << pose.header.stamp.toSec() << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.position.x << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.position.y << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.position.z << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.orientation.x << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.orientation.y << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.orientation.z << ","
//                      << std::fixed << std::setprecision(6) << pose.pose.orientation.w << "\n";
//         }
//         csv_file.close();
//         res.success = true;
//     } else {
//         ROS_ERROR("Failed to open file %s for writing", filename.c_str());
//         res.success = false;
//     }

//     return res.success;
// }



// void TrajectoryPublisherSaver::publishMarkers(const visualization_msgs::MarkerArray& markers) {
//     marker_pub_.publish(markers);
// }
