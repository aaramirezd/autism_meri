
#include <string>
#include <ros/ros.h>

#include "local_face_tracking_node.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "local_tracker");
    ros::NodeHandle _private_node("~");
    ros::Rate hz(30);
    // load parameters
    string arguments;
    
    _private_node.getParam("model_loc", arguments);
    
    stringstream ss(arguments);
    istream_iterator<string> begin(ss);
    istream_iterator<string> end;
    std::vector<string> v_arguments(begin, end);
    
    ROS_INFO_STREAM("Initializing the face detector");
    TrackerEstimator estimator(_private_node,v_arguments);
    ROS_INFO("Tracker_estimator is ready");
    ros::spin();

    return 0;
}

