#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <list>
#include <sstream>
#include <fstream>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

//#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <Eigen/StdVector>

// Global variables:

std::vector<geometry_msgs::PoseStamped> poses_vector;
ros::Time latest_time;
std::string latest_source="";
std::string latest_best="";
std::map<std::string, ros::Time> last_received_detection_;
std::map<std::string, ros::Duration> delay_between_s_detection_;
ros::Duration max_time_between_detections_;
geometry_msgs::PoseStamped best_pose;
geometry_msgs::PoseStamped best_pose_old;
geometry_msgs::TransformStamped best_pose_tr;
std::map<std::string, geometry_msgs::TransformStamped> cameras_transformStamped;
EigenSTL::vector_Affine3d rs(0);
bool is_pose_valid=false;
//***********************           Average Quaternion Estimation ********************************************************
Eigen::Quaterniond average(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& qs)
{
  Eigen::MatrixXd Q (4, qs.size());

  for (std::size_t i = 0; i < qs.size(); ++i)
  {
    Q.col(i) = qs[i].coeffs();
  }

  Eigen::MatrixXd Q_prime = Q * Q.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(Q_prime);

  Eigen::VectorXd eigen_vals = eigensolver.eigenvalues();
  Eigen::MatrixXd eigen_vecs = eigensolver.eigenvectors();

  int max_idx = 0;
  double max_value = 0.0;
  for (int i = 0; i < eigen_vals.size(); ++i)
  {
    if (eigen_vals(i) > max_value)
    {
      max_idx = i;
      max_value = eigen_vals(i);
    }
  }

  Eigen::VectorXd coeffs = eigen_vecs.col(max_idx);
  Eigen::Quaterniond avg_quat (coeffs(3), coeffs(0), coeffs(1), coeffs(2));
  return avg_quat;
}

Eigen::Affine3d averagePose(const EigenSTL::vector_Affine3d& poses)
{
  Eigen::Affine3d avg_pose;
  Eigen::Vector3d avg_trans(0.0,0.0,0.0);
  Eigen::Quaterniond avg_quaternion(0.0,0.0,0.0,1.0);
  if(poses.size() != 0){
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > qs;
    qs.reserve(poses.size());
    //ROS_INFO_STREAM("N poses: " << poses.size());
    for (const auto& p : poses)
    {
      qs.push_back(Eigen::Quaterniond(p.rotation()));
      avg_trans += p.translation();
    }
    avg_trans /= poses.size();
    avg_quaternion = average(qs);
  }
  avg_pose.translation() = avg_trans;
  avg_pose.linear() = avg_quaternion.toRotationMatrix();//poses[0].linear();//
  return avg_pose;
}
//***********************    Timer Callback ********************************************************
void timer_callback(const ros::TimerEvent& event)
{
  Eigen::Affine3d avg_affine_pose = averagePose(rs);
  if (avg_affine_pose.translation().x() != 0 & avg_affine_pose.translation().y() != 0){
  best_pose_tr = tf2::eigenToTransform(avg_affine_pose);
  }
  //ROS_INFO_STREAM("Time timer callback: " << ros::Time::now() <<"vector size: " << rs.size());
  rs.clear();
}
//***********************    Pose Callback ********************************************************
void call_global_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// Read message header information:
  std::string frame_id = msg->header.frame_id;
  ros::Time frame_time = msg->header.stamp;

  std::string frame_id_tmp = frame_id;
  int pos = frame_id_tmp.find("_rgb_optical_frame");
  if (pos != std::string::npos){
    frame_id_tmp.replace(pos, std::string("_rgb_optical_frame").size(), "");
  }
  pos = frame_id_tmp.find("_ir_optical_frame");
  if (pos != std::string::npos){
    frame_id_tmp.replace(pos, std::string("_ir_optical_frame").size(), "");
  }
  Eigen::Affine3d e;
  Eigen::Affine3d e_wrtw;
  tf2::convert(msg->pose, e);
  tf2::doTransform(e, e_wrtw, cameras_transformStamped[frame_id_tmp]);//doTransform(const Eigen::Affine3d& t_in,Eigen::Affine3d& t_out,const geometry_msgs::TransformStamped& transform)
  
  rs.push_back(e_wrtw);
  //ROS_INFO_STREAM("Time pose callback: " << ros::Time::now()-latest_time <<"vector size: " << rs.size());
  latest_time = ros::Time::now();
  is_pose_valid=true;
}
//***********************    Pose Callback ********************************************************
int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_estimation");
  ros::NodeHandle nh("~");

  ros::Subscriber input_sub = nh.subscribe("input", 5, call_global_pose);
  ros::Timer timer = nh.createTimer(ros::Duration(0.066), timer_callback);

  static tf2_ros::TransformBroadcaster pose_broadcaster;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  
  std::string face_frame;
  nh.param("face_frame", face_frame, std::string("focus_frame"));
       
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "face_link";
  static_transformStamped.child_frame_id = face_frame;
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 3.14159/2.0 ,0.0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  bool pub_static=true;
  
  int num_cameras;
  nh.param("N_cameras", num_cameras, 2);
  std::vector<std::string> cameras_name;
  cameras_name.push_back("kinect2_a");
  cameras_name.push_back("kinect2_b");
  cameras_name.push_back("kinect2_c");
  
  double rate;
  nh.param("rate", rate, 30.0);
  double max_time_between_detections_d;
  nh.param("max_time_between_detections", max_time_between_detections_d, 1.0);
  max_time_between_detections_ = ros::Duration(max_time_between_detections_d);
  
  ros::Rate hz(num_cameras*rate);
  
  std::map<std::string, ros::Time> last_message;
  
  for (int i = 0; i < num_cameras; ++i){
    try{
        cameras_transformStamped[cameras_name[i]] = tfBuffer.lookupTransform("world", cameras_name[i], ros::Time::now(), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex){
          ROS_WARN("%s",ex.what());
    }
  }
  //ROS_INFO_STREAM("N cameras: " << cameras_transformStamped.size());
  bool isready;
  while (!ros::param::get("/user_is_ready", isready))
  {
    ROS_INFO_ONCE("Waiting for nodes initialization");
  }
  while (ros::ok())
  {
    ros::spinOnce();
    if(is_pose_valid){  
        best_pose_tr.header.frame_id = "world";
        best_pose_tr.child_frame_id = "face_link";
        best_pose_tr.header.stamp = ros::Time::now();
        pose_broadcaster.sendTransform(best_pose_tr);
        if(pub_static){
          static_broadcaster.sendTransform(static_transformStamped);     
          pub_static=!pub_static;
        }
    }
    hz.sleep();
  }
  return 0;
}
