
#include <string>
#include <set>

//OpenFace library
#include "LandmarkCoreIncludes.h"
#include "GazeEstimation.h"

#include <SequenceCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>
#include <RotationHelpers.h>

#include <fstream>
#include <sstream>

// opencv2
#ifdef OPENCV3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

// ROS
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "meri_local/Reset.h"

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>



// dlib
#include <iostream>
#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

/*
#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort( const std::string & error )
{
    std::cout << error << std::endl;
    abort();
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )
*/
using namespace std;
using namespace dlib;

template <long num_filters, typename SUBNET> using con5d = con<num_filters,5,5,2,2,SUBNET>;
template <long num_filters, typename SUBNET> using con5  = con<num_filters,5,5,1,1,SUBNET>;

template <typename SUBNET> using downsampler  = relu<affine<con5d<32, relu<affine<con5d<32, relu<affine<con5d<16,SUBNET>>>>>>>>>;
template <typename SUBNET> using rcon5  = relu<affine<con5<45,SUBNET>>>;

using net_type = loss_mmod<con<1,9,9,1,1,rcon5<rcon5<rcon5<downsampler<input_rgb_image_pyramid<pyramid_down<6>>>>>>>>;
// ----------------------------------------------------------------------------------------
template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual = add_prev1<block<N,BN,1,tag1<SUBNET>>>;

template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual_down = add_prev2<avg_pool<2,2,2,2,skip1<tag2<block<N,BN,2,tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET> 
using block  = BN<con<N,3,3,1,1,relu<BN<con<N,3,3,stride,stride,SUBNET>>>>>;

template <int N, typename SUBNET> using ares      = relu<residual<block,N,affine,SUBNET>>;
template <int N, typename SUBNET> using ares_down = relu<residual_down<block,N,affine,SUBNET>>;

template <typename SUBNET> using alevel0 = ares_down<256,SUBNET>;
template <typename SUBNET> using alevel1 = ares<256,ares<256,ares_down<256,SUBNET>>>;
template <typename SUBNET> using alevel2 = ares<128,ares<128,ares_down<128,SUBNET>>>;
template <typename SUBNET> using alevel3 = ares<64,ares<64,ares<64,ares_down<64,SUBNET>>>>;
template <typename SUBNET> using alevel4 = ares<32,ares<32,ares<32,SUBNET>>>;

using anet_type = loss_metric<fc_no_bias<128,avg_pool_everything<
                            alevel0<
                            alevel1<
                            alevel2<
                            alevel3<
                            alevel4<
                            max_pool<3,3,2,2,relu<affine<con<32,7,7,2,2,
                            input_rgb_image_sized<150>
                            >>>>>>>>>>>>;    
class TrackerEstimator
{
public:

    TrackerEstimator(ros::NodeHandle& rosNode, std::vector<string> &arguments);
    
    
private: 
    // ----------------------------------------------------------------------------------------

    
    net_type net_det;
    anet_type net;
    shape_predictor sp;
    std::vector<matrix<float,0,1>> know_face_descriptor;
    // ----------------------------------------------------------------------------------------
    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    ros::Publisher gazeDirection_pub;
    ros::Publisher pose_pub;
    ros::ServiceServer reset_service;
    geometry_msgs::PoseStamped local_pose;
    
    cv_bridge::CvImagePtr cv_ptr;
 
    cv::Mat processing_image;
    cv::Mat croppedImg;
    cv::Mat know_image;
    cv::Rect_<float> region;
    cv::Vec6d pose_estimate;
  
    matrix<rgb_pixel> dlib_img;
    std::vector<correlation_tracker> unknow_dlib_trackers;
    std::vector<correlation_tracker> know_dlib_trackers;
    dlib::drectangle know_tracker_rec;
        
    // OpenFace
    LandmarkDetector::FaceModelParameters det_parameters;
    LandmarkDetector::CLNF clnf_model; 

    
    std::vector<cv::Rect> faces;
    std::vector<matrix<rgb_pixel>> Rec_faces;
    std::string parent_path;
    //HeadPoseEstimation estimator;

    bool tracker_init;
    bool user_init;
    bool detection_success;
    
    int frame_count;
    double detection_certainty;

    Utilities::Visualizer visualizer;
    Utilities::FpsTracker fps_tracker;
    Utilities::SequenceCapture sequence_reader;

    void detectFaces(const sensor_msgs::ImageConstPtr& msg);
    void dnn_detect_Faces(const sensor_msgs::ImageConstPtr& msg);
    void read_know_face();
    bool reset_tracker(meri_local::Reset::Request  &req, meri_local::Reset::Response &res);
};

