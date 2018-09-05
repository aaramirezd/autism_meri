#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#ifdef OPENCV3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

cv::Mat croppedImg;
bool image_captured=false;
int box_size=0;
std::string FullPath;
std::string to_FullPath;

static  void onMouse(int ev, int x, int y, int, void*)
{
  if( ev == EVENT_LBUTTONDOWN ){      
      cv::Mat face_user_img;
      croppedImg(cv::Rect(x-(box_size/2),y-(box_size/2),box_size,box_size)).copyTo(face_user_img);
      cv::imwrite( FullPath, face_user_img );
      ROS_INFO(" Image has been saved ");
      image_captured=true;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(croppedImg);
    cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("Display", onMouse, 0);
    cv::imshow("Display", croppedImg);
    cv::waitKey(20);
}

int main( int argc, char** argv )
{   
    ros::init(argc, argv, "init_child");
    ros::NodeHandle n("~");
    ros::Rate r(30);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub;
    n.param("box_size", box_size, 250);
    n.param("FullPath", FullPath, std::string(" "));
    n.param("to_FullPath", to_FullPath, std::string(" "));
    
    sub = it.subscribe("image", 1, imageCallback);
    while (!image_captured)
    {
      ros::spinOnce();
      r.sleep();
    }
    boost::filesystem::copy_file(FullPath, to_FullPath, boost::filesystem::copy_option::overwrite_if_exists);
    n.setParam("/user_is_ready", true);      
    cv::destroyAllWindows();
}
