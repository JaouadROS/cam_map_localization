#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp> 
#include <string>
#include <aruco.h>

class MarkerMapPoseEstimation
{
    aruco::CameraParameters TheCameraParameters;  
    aruco::MarkerMap TheMarkerMapConfig;	 
    aruco::MarkerDetector TheMarkerDetector;
    float TheMarkerSize;
    aruco::MarkerMapPoseTracker TheMSPoseTracker;
    cv::Mat pose_map;
    std::string TheMarkerMapConfig_path;
    std::string TheCameraParameters_path;

public:
    MarkerMapPoseEstimation();
    void callBackColor(const sensor_msgs::ImageConstPtr&);
    geometry_msgs::PoseStamped frameToRosPose(const cv::Mat&);
    void getQuaternionAndTranslationfromMatrix44(const cv::Mat&, float&, float&, float&, float&, float&, float&, float&);

protected:    
    ros::NodeHandle nh_node;
    ros::Subscriber sub;
    ros::Publisher poseStampedPub;
};

