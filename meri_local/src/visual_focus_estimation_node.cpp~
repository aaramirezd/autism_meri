#include <string>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
static const double FOV = 25. / 180 * M_PI; // radians
static const float RANGE = 3.5; //m

visualization_msgs::Marker makeMarker(const string& frame) {
    static std_msgs::ColorRGBA Purple;
    Purple.r = 0.5; Purple.g = 0.; Purple.b = 0.5; Purple.a = 1.;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "focus_of_attention";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.10;
    marker.scale.y = 0.10;
    marker.scale.z = 0.10;
    marker.color = Purple;
    marker.lifetime = ros::Duration(2);
    return marker;
}

bool isInFieldOfView(const tf2_ros::Buffer& listener, const string& target_frame, const string& observer_frame) {

    geometry_msgs::TransformStamped transform;

    try{
      transform = listener.lookupTransform(observer_frame, target_frame, ros::Time::now(), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex){
      ROS_WARN("%s",ex.what());
    }
    if (transform.transform.translation.x < 0) return false;
   
    double distance_to_main_axis = sqrt(transform.transform.translation.y * transform.transform.translation.y + transform.transform.translation.z * transform.transform.translation.z);
    double fov_radius_at_x = tan(FOV/2) * transform.transform.translation.x;
    
    if (distance_to_main_axis < fov_radius_at_x) return true;
    return false;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "estimate_focus");
    ros::NodeHandle n("~");
    ros::Rate r(30);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("estimate_focus", 1);
    ros::Publisher fov_pub = n.advertise<sensor_msgs::Range>("face_field_of_view", 1);
    ros::Publisher frames_in_fov_pub = n.advertise<std_msgs::Header>("actual_focus_of_attention", 1);

    std::string face_frame;
    n.param("face_frame", face_frame, std::string("focus_frame"));
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped focus_frame_tr;

    vector<string> monitored_frames = {"trem", "caminhao", "helicoptero", "robo"};

    std_msgs::Header fov_header;
    sensor_msgs::Range fov;
    
    fov.radiation_type = sensor_msgs::Range::INFRARED;
    fov.field_of_view = FOV;
    fov.min_range = 0;
    fov.max_range = 10;
    fov.range = RANGE;
    bool face_visible=false;
    ROS_INFO("Waiting until a face becomes visible...");
    bool isready;
    while (!ros::param::get("/user_is_ready", isready))
    {
      ROS_INFO_ONCE("Waiting for nodes initialization");
    }
    while (!face_visible) {
      try{
        focus_frame_tr = tfBuffer.lookupTransform("world", face_frame, ros::Time::now(), ros::Duration(3.0));
        face_visible=true;
      }
      catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
      }
      r.sleep();
    }                       
    ROS_INFO("Face detected! We can start estimating the focus of attention...");
    
    while (ros::ok())
    {
      for(auto frame : monitored_frames) {
        if(isInFieldOfView(tfBuffer, frame, face_frame)) {
          fov_header.frame_id=frame;
          fov_header.stamp=ros::Time::now();
          marker_pub.publish(makeMarker(frame));
          frames_in_fov_pub.publish(fov_header);
        }  
      }
      fov.range = RANGE;
      fov.header.stamp = ros::Time::now();
      fov.header.frame_id = "focus_frame";
      fov_pub.publish(fov); 
      r.sleep();  
    }
}
