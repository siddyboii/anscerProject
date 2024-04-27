#include "trajectory_visualization_storage/TrajectoryReaderPublisher.h"
#include "ros/ros.h"

namespace trajectory_visualization_storage {

TrajectoryReaderPublisher::TrajectoryReaderPublisher()
    : nh_("~"),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>("visualization_marker_array", 1)) {
    ros::NodeHandle private_nh("~");
    private_nh.param("filename", filename_, std::string("trajectory.csv"));

    trajectory_.clear();

    // Create the timer
    timer_t = nh_.createTimer(ros::Duration(1.0), &TrajectoryReaderPublisher::onTimer, this);
}

TrajectoryReaderPublisher::~TrajectoryReaderPublisher() {}

void TrajectoryReaderPublisher::onTimer(const ros::TimerEvent&) {
    // Load the trajectory from a file
    std::ifstream in(filename_);
    if (in.is_open()) {
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream iss(line);
            double x, y, z, ox, oy, oz, ow;
            iss >> x >> y >> z >> ox >> oy >> oz >> ow;

            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = ox;
            pose.orientation.y = oy;
            pose.orientation.z = oz;
            pose.orientation.w = ow;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory";
            marker.id = trajectory_.size();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            trajectory_.push_back(marker);
        }
        in.close();
        ROS_INFO("File opened");
    } else {
        ROS_ERROR("Failed to open file %s for reading.", filename_.c_str());
    }

    visualization_msgs::Marker marker_array;
    // marker_array.markers = trajectory_.markers;
    marker_pub_.publish(marker_array);
    
}

void TrajectoryReaderPublisher::loadTrajectory() {
    onTimer(ros::TimerEvent());
}

} // namespace trajectory_visualization_storage

int main(int argc, char **argv){
    ros::init(argc,argv, "trajectory_reader_publisher");
    trajectory_visualization_storage::TrajectoryReaderPublisher trajectory_reader_publisher;
    ros::spin();
    return 0;
}






















// #include "trajectory_visualization_storage/TrajectoryReaderPublisher.h"

// TrajectoryReaderPublisher::TrajectoryReaderPublisher()
//     : nh_("~"),
//       marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1)) {
//     ros::NodeHandle private_nh("~");
//     private_nh.param("filename", filename_, std::string("trajectory.csv"));

//     std::ifstream file(filename_);
//     if (!file.is_open()) {
//         ROS_ERROR("Failed to open file %s for reading.", filename_.c_str());
//         return;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         std::istringstream iss(line);
//         double x, y, z, qx, qy, qz, qw, timestamp;
//         if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw >> timestamp)) {
//             ROS_ERROR("Failed to parse line: %s", line.c_str());
//             continue;
//         }

//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time(timestamp);
//         pose.header.frame_id = "map";
//         pose.pose.position.x = x;
//         pose.pose.position.y = y;
//         pose.pose.position.z = z;
//         pose.pose.orientation.x = qx;
//         pose.pose.orientation.y = qy;
//         pose.pose.orientation.z = qz;
//         pose.pose.orientation.w = qw;

//         marker_.markers.push_back(pose);
//     }

//     file.close();

//     marker_.markers[0].id = 0;
//     marker_.markers[0].type = visualization_msgs::Marker::SPHERE;
//     marker_.markers[0].action = visualization_msgs::Marker::ADD;
//     marker_.markers[0].scale.x = 0.1;
//     marker_.markers[0].scale.y = 0.1;
//     marker_.markers[0].scale.z = 0.1;
//     marker_.markers[0].color.r = 1.0;
//     marker_.markers[0].color.g = 0.0;
//     marker_.markers[0].color.b = 0.0;
//     marker_.markers[0].color.a = 1.0;

//     marker_pub_.publish(marker_);
// }

// void TrajectoryReaderPublisher::run() {
//     ros::spin();
// }

