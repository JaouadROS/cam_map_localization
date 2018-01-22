
#include "cam_map_localization/pose_map_estimation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_map_localization");

  MarkerMapPoseEstimation cam_image_map;

  ros::spin();
  return 0;
}
