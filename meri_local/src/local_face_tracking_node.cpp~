#include "local_face_tracking_node.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the face transformation should be published?
// this allow to compensate for the 'slowness' of face detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0.0

TrackerEstimator::TrackerEstimator(ros::NodeHandle& rosNode, std::vector<string> &arguments) :
            rosNode(rosNode),
            it(rosNode),
            det_parameters(arguments),
            clnf_model(det_parameters.model_location),
            visualizer(true, false, false, false)
{
    bool isready;
    while (!ros::param::get("/user_is_ready", isready))
    {
      ROS_INFO_ONCE("Waiting for nodes initialization");
    }
    
    sub = it.subscribe("image", 1, &TrackerEstimator::dnn_detect_Faces,this);
    pub = it.advertise("face",1);
    reset_service = rosNode.advertiseService("reset_face_tracker", &TrackerEstimator::reset_tracker,this);
    pose_pub = rosNode.advertise<geometry_msgs::PoseStamped>("/local_pose", 3);
  
    sequence_reader.Open(arguments);
    det_parameters.quiet_mode = true;
    ros::Rate hz(30);
    fps_tracker.AddFrame();
    rosNode.param("parent_path", parent_path, std::string("/home/castor2/catkin_ws/src/autism_meri/meri_local"));
    deserialize(parent_path + "/models/mmod_human_face_detector.dat") >> net_det;
    deserialize(parent_path + "/models/shape_predictor_5_face_landmarks.dat") >> sp;
    deserialize(parent_path + "/models/dlib_face_recognition_resnet_model_v1.dat") >> net;
    
    tracker_init=false;
    user_init=false;
}

bool TrackerEstimator::reset_tracker(meri_local::Reset::Request  &req, meri_local::Reset::Response &res)
{
    if (req.a == 2){
    clnf_model.Reset();
    }
    tracker_init=false;
    ROS_INFO("Tracker reseted");
    res.b=req.a;
    return true;
}
void TrackerEstimator::read_know_face()
{
      matrix<rgb_pixel> know_img; 
      array2d<rgb_pixel> know_cimg;
      load_image(know_cimg, parent_path + "/data/user.jpg");
      assign_image(know_img, know_cimg);
      
      auto know_det = net_det(know_img);
      auto know_shape = sp(know_img, know_det[0]);
      matrix<rgb_pixel> know_face_chip;
      extract_image_chip(know_img, get_face_chip_details(know_shape,150,0.25), know_face_chip);
      std::vector<matrix<rgb_pixel>> know_rec_faces;
      know_rec_faces.push_back(move(know_face_chip));
      know_face_descriptor = net(move(know_rec_faces));
      user_init = true;
      frame_count = 0;
}
 
void TrackerEstimator::dnn_detect_Faces(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(croppedImg);
    cv::Mat_<uchar> grayscale_image;
    
    if(croppedImg.channels() == 3)
    {
        cv::cvtColor(croppedImg, grayscale_image, CV_RGB2GRAY);				
    }
    else
    {
        grayscale_image = croppedImg.clone();				
    }
    
    if (!user_init){
        read_know_face();
    }
    else{
        cv_image<bgr_pixel> cimg(croppedImg);
        assign_image(dlib_img, cimg); 
        if (!tracker_init){
            auto dets = net_det(dlib_img);

            faces.clear();
            Rec_faces.clear();
            for (auto&& d : dets){
                //l = x //t = y //r = x+w //b = y+h
                auto shape = sp(dlib_img, d);
                matrix<rgb_pixel> face_chip;
                extract_image_chip(dlib_img, get_face_chip_details(shape,150,0.25), face_chip);
                Rec_faces.push_back(move(face_chip));
                cv::Rect newFace((d.rect.left()), //x
                     (d.rect.top()), //y
                     (d.rect.right()-d.rect.left()), //width
                     (d.rect.bottom()-d.rect.top())  //height
                ); // height
                faces.push_back(newFace);
            }
            std::vector<matrix<float,0,1>> face_descriptors = net(Rec_faces);
            std::vector<std::string> label;
            unknow_dlib_trackers.clear();
            
            for (size_t i = 0; i < face_descriptors.size(); ++i){
                ROS_INFO_STREAM(" N of Faces "<< face_descriptors.size());
                if (length(face_descriptors[i]-know_face_descriptor[0]) < 0.6){
                    label.push_back("face_know");
                    if(dets[i].rect.width()!= 0){
                        dlib_tracker.start_track(dlib_img, dets[i].rect);
                        know_tracker_rec=dets[i].rect;
                    }
                }
                else{
                    label.push_back("face_unknow");
                    correlation_tracker temp_dlib_tracker;
                    if(dets[i].rect.width()!= 0){
                        temp_dlib_tracker.start_track(dlib_img, dets[i].rect);
                        unknow_dlib_trackers.push_back(temp_dlib_tracker);
                    }
                }
            }
            tracker_init = true;
            ROS_INFO(" Dlib tracker activated ");
        }
        
        for (auto&& tr : unknow_dlib_trackers){
            tr.update(dlib_img);
            dlib::drectangle unknow_tracker_rec=tr.get_position();
            cv::Rect_<float> temp_rect;
            temp_rect.x = unknow_tracker_rec.tl_corner().x();
            temp_rect.y = unknow_tracker_rec.tl_corner().y();
            temp_rect.width = unknow_tracker_rec.width();
            temp_rect.height = unknow_tracker_rec.height();
            cv::rectangle(croppedImg,temp_rect,CV_RGB(255, 255 , 255),-1);
        }
        
        dlib_tracker.update(dlib_img);
        know_tracker_rec=dlib_tracker.get_position();	      
        
	    region.x = know_tracker_rec.tl_corner().x();// + 0.0389f * know_tracker_rec.width();
	    region.y = know_tracker_rec.tl_corner().y();// + 0.1278f * know_tracker_rec.height();
    	region.width = know_tracker_rec.width();// * 0.9611);
        region.height = know_tracker_rec.height();// * 0.9388);
        
        cv::rectangle(croppedImg,region,CV_RGB(50, 255 , 50),2);
        
        fps_tracker.AddFrame();
        
        detection_success = LandmarkDetector::DetectLandmarksInVideo(croppedImg, clnf_model, det_parameters, grayscale_image);          
        cv::Rect_<float> bounding_box;
        clnf_model.pdm.CalcBoundingBox(bounding_box, clnf_model.params_global, clnf_model.params_local);
        cv::rectangle(croppedImg,bounding_box,CV_RGB(50, 255 , 255),2);
        visualizer.SetImage(croppedImg, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
        visualizer.SetFps(fps_tracker.GetFPS(),2);
        detection_certainty = clnf_model.detection_certainty;
        
        if (detection_success){
            pose_estimate = LandmarkDetector::GetPose(clnf_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
            visualizer.SetObservationLandmarks(clnf_model.detected_landmarks, clnf_model.detection_certainty, clnf_model.GetVisibilities());
	        visualizer.SetObservationPose(pose_estimate, clnf_model.detection_certainty);
        }
        //ROS_INFO_STREAM("H_x: " << float(pose_estimate[0]/1000.0) << ", H_y: " << float(pose_estimate[1]/1000.0) << ", H_z: " << float(pose_estimate[2]/1000.0));
        tf::Quaternion q_new; 
        q_new = tf::createQuaternionFromRPY(pose_estimate[3], pose_estimate[4], pose_estimate[5]);
        //Set the quaternion using fixed axis RPY, roll Angle around X, pitch Angle around Y, yaw Angle around Z
        q_new.normalize();
        
        if(pose_pub.getNumSubscribers() > 0){
            local_pose.header.stamp = ros::Time::now();
            local_pose.header.frame_id = msg->header.frame_id;
            local_pose.pose.position.x = float(pose_estimate[0]/1000.0);
            local_pose.pose.position.y = float(pose_estimate[1]/1000.0);
            local_pose.pose.position.z = float(pose_estimate[2]/1000.0);
            quaternionTFToMsg(q_new, local_pose.pose.orientation);
            pose_pub.publish(local_pose);    
        }    
        if(pub.getNumSubscribers() > 0){
            processing_image = visualizer.GetVisImage();
            ROS_INFO_ONCE("Starting to publish face tracking output for debug");
            std_msgs::Header header_i;
            header_i.stamp=ros::Time::now();
            header_i.frame_id=msg->header.frame_id;
            pub.publish(cv_bridge::CvImage(header_i, sensor_msgs::image_encodings::BGR8, processing_image).toImageMsg());
        }
    }
}

void TrackerEstimator::detectFaces(const sensor_msgs::ImageConstPtr& msg)
{   
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat_<uchar> grayscale_image;
    cv_ptr->image.copyTo(croppedImg);
       
    if(croppedImg.channels() == 3)
    {
        cv::cvtColor(croppedImg, grayscale_image, CV_RGB2GRAY);				
    }
    else
    {
        grayscale_image = croppedImg.clone();				
    }
      
    detection_success = LandmarkDetector::DetectLandmarksInVideo(croppedImg, clnf_model, det_parameters, grayscale_image);                      
    
    detection_certainty = clnf_model.detection_certainty;
        
    // Gaze tracking, absolute gaze direction
    cv::Point3f gazeDirection0(1, 0, 0);
    cv::Point3f gazeDirection1(0, 0, -1);
  
    if (detection_success){
    
            cv::Vec6d pose_estimate = LandmarkDetector::GetPose(clnf_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
            fps_tracker.AddFrame();
            
            visualizer.SetImage(croppedImg, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
	        visualizer.SetObservationLandmarks(clnf_model.detected_landmarks, clnf_model.detection_certainty, clnf_model.GetVisibilities());
	        visualizer.SetObservationPose(pose_estimate, clnf_model.detection_certainty);
            visualizer.SetFps(fps_tracker.GetFPS(),2);
            
            processing_image = visualizer.GetVisImage();  
    }
                
    if(pub.getNumSubscribers() > 0) 
    {
    ROS_INFO_ONCE("Starting to publish face tracking output for debug");
    std_msgs::Header header_i;
    header_i.stamp=ros::Time::now();
    header_i.frame_id=msg->header.frame_id;
    pub.publish(cv_bridge::CvImage(header_i, msg->encoding, processing_image).toImageMsg());
    }
    
}

