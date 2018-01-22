#include "cam_map_localization/pose_map_estimation.h"

MarkerMapPoseEstimation::MarkerMapPoseEstimation() : nh_node("~")
{
  nh_node.param<std::string>("TheMarkerMapConfig_path", TheMarkerMapConfig_path, "");
  TheMarkerMapConfig.readFromFile(TheMarkerMapConfig_path);

  nh_node.param<std::string>("TheCameraParameters_path", TheCameraParameters_path, "");
  TheCameraParameters.readFromXMLFile(TheCameraParameters_path);

  TheMarkerDetector.setDictionary( TheMarkerMapConfig.getDictionary());
  nh_node.param<float>("TheMarkerSize", TheMarkerSize, 0.158);

  if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0)
       TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);
  
  if (TheCameraParameters.isValid() && TheMarkerMapConfig.isExpressedInMeters())
       TheMSPoseTracker.setParams(TheCameraParameters, TheMarkerMapConfig);

  
  poseStampedPub = nh_node.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 1, true);				
  sub = nh_node.subscribe("image_raw", 1, &MarkerMapPoseEstimation::callBackColor, this);
}
 

void MarkerMapPoseEstimation::callBackColor(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
        
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(cv_ptr->image);

  if (TheMSPoseTracker.isValid())
    if (TheMSPoseTracker.estimatePose(detected_markers)) 
      {
         pose_map=TheMSPoseTracker.getRTMatrix();
	 pose_map = pose_map.inv();
      }
   
   geometry_msgs::PoseStamped poseStamped = frameToRosPose(pose_map);

   for(auto idx : TheMarkerMapConfig.getIndices(detected_markers))
          detected_markers[idx].draw(cv_ptr->image, cv::Scalar(0, 0, 255), 1);

   poseStampedPub.publish(poseStamped);

   cv::imshow("Image with markers detected", cv_ptr->image);
   cv::waitKey(1) ;
  
}


void MarkerMapPoseEstimation::getQuaternionAndTranslationfromMatrix44(const cv::Mat& M_in, float& qx, float& qy, float& qz, float& qw, float& tx, float& ty, float& tz)
{
    auto SIGN=[](float x) {return (x >= 0.0f) ? +1.0f : -1.0f;};
    auto NORM=[](double a, double b, double c, double d) {return sqrt(a * a + b * b + c * c + d * d);};

    assert(M_in.total() == 16);

    cv::Mat M;
    M_in.convertTo(M, CV_32F);

    float r11 = M.at<float>(0, 0);
    float r12 = M.at<float>(0, 1);
    float r13 = M.at<float>(0, 2);
    float r21 = M.at<float>(1, 0);
    float r22 = M.at<float>(1, 1);
    float r23 = M.at<float>(1, 2);
    float r31 = M.at<float>(2, 0);
    float r32 = M.at<float>(2, 1);
    float r33 = M.at<float>(2, 2);

    double q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
    double q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
    double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;

    if (q0 < 0.0f)
        q0 = 0.0f;

    if (q1 < 0.0f)
        q1 = 0.0f;

    if (q2 < 0.0f)
        q2 = 0.0f;

    if (q3 < 0.0f)
        q3 = 0.0f;

    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);

    if (q0 >= q1 && q0 >= q2 && q0 >= q3)
    {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }

    else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
    {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }

    else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
    {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    }

    else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
    {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    }

    else
    {
        std::cout << "Coding error" << std::endl;
    }

    double r = NORM(q0, q1, q2, q3);

    qx = static_cast<float>(q0 / r);
    qy = static_cast<float>(q1 / r);
    qz = static_cast<float>(q2 / r);
    qw = static_cast<float>(q3 / r);

    tx = M.at<float>(0, 3);
    ty = M.at<float>(1, 3);
    tz = M.at<float>(2, 3);

}

geometry_msgs::PoseStamped MarkerMapPoseEstimation::frameToRosPose(const cv::Mat& fmp)
{
   float qx, qy, qz, qw, tx, ty, tz;
   geometry_msgs::PoseStamped poseStamped;
   if (!fmp.empty())
    {
      getQuaternionAndTranslationfromMatrix44(fmp, qx, qy, qz, qw, tx, ty, tz);

      poseStamped.header.frame_id="/map";
      poseStamped.header.stamp = ros::Time::now();

      poseStamped.pose.position.x = tx;
      poseStamped.pose.position.y = ty;
      poseStamped.pose.position.z = tz;
      poseStamped.pose.orientation.x =- qz;
      poseStamped.pose.orientation.y = qy;
      poseStamped.pose.orientation.z = -qx;
      poseStamped.pose.orientation.w = qw;
    }
   return poseStamped;
}



































