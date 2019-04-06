#include <turtlebot_moving_obstacles/aruco_follower.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_follower");

  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  ArucoFollower af;

  af.initialize(n, nh_private);

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    af.updatePose();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}