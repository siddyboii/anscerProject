#ifndef TRAJECTORY_READER_PUBLISHER_H
#define TRAJECTORY_READER_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>

namespace trajectory_visualization_storage {

class TrajectoryReaderPublisher {
public:
    TrajectoryReaderPublisher();
    ~TrajectoryReaderPublisher();

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Timer timer_t;
    std::string filename_;
    std::vector<visualization_msgs::Marker> trajectory_;
    

    void loadTrajectory();
    void onTimer(const ros::TimerEvent&);
};

} // namespace trajectory_visualization_storage

#endif // TRAJECTORY_READER_PUBLISHER_H




















// #ifndef TRAJECTORY_READER_PUBLISHER_H
// #define TRAJECTORY_READER_PUBLISHER_H

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <fstream>
// #include <string>

// class TrajectoryReaderPublisher {
// public:
//     TrajectoryReaderPublisher();
//     void run();

// private:
//     ros::NodeHandle nh_;
//     ros::Publisher marker_pub_;
//     visualization_msgs::MarkerArray marker_;
//     std::string filename_;
// };

// #endif // TRAJECTORY_READER_PUBLISHER_H

