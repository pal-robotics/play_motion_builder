#include "play_motion_builder.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "play_motion_builder_node");
  pal::ROSMotionBuilderNode rmbn;

  if (rmbn.initialize())
    ros::spin();

  return 0;
}
