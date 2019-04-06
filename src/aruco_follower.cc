#include <turtlebot_moving_obstacles/aruco_follower.h>

const int ArucoFollower::DIST_FROM_TARGET = 0.2;

void ArucoFollower::initialize(ros::NodeHandle &nh,
                               ros::NodeHandle &private_nh) {
  private_nh.getParam("lin_vel_scale", lin_vel_scale_);
  private_nh.getParam("ang_vel_scale", ang_vel_scale_);

  cmdpub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  markerpub_ =
      private_nh.advertise<visualization_msgs::Marker>("aruco_marker", 1);
  sub_ = nh.subscribe<marker_msgs::MarkerDetection>(
      "markers", 1, &ArucoFollower::markercb, this);
}

bool ArucoFollower::updatePose() {
  tf::StampedTransform transform;
  try {
    transform_listener_.lookupTransform("t4", "base_link", ros::Time(0),
                                        transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  robot_pose_.pose.position.x = transform.getOrigin().x();
  robot_pose_.pose.position.y = transform.getOrigin().y();
  robot_pose_.pose.position.z = transform.getOrigin().z();
  robot_pose_.pose.orientation.x = transform.getRotation().x();
  robot_pose_.pose.orientation.y = transform.getRotation().y();
  robot_pose_.pose.orientation.z = transform.getRotation().z();
  robot_pose_.pose.orientation.w = transform.getRotation().w();
  return true;
}

void ArucoFollower::markercb(const marker_msgs::MarkerDetectionConstPtr &msg) {
  if (msg->markers.empty()) {
    ROS_INFO_THROTTLE(1, "No marker info. Returning..");
    cmd_.linear.x = 0;
    cmd_.angular.z = 0;
    cmdpub_.publish(cmd_);
    return;
  }

  geometry_msgs::Pose marker_pose = msg->markers[0].pose;
  tf::Quaternion q(
      robot_pose_.pose.orientation.x, robot_pose_.pose.orientation.y,
      robot_pose_.pose.orientation.z, robot_pose_.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  geometry_msgs::Point origin;
  double ang_dist =
      angles::normalize_angle(getAngle(marker_pose.position, origin));
  double lin_dist = getDistance(robot_pose_.pose.position, origin);

  cmd_.linear.x = lin_dist * lin_vel_scale_;
  cmd_.angular.z = -ang_dist * ang_vel_scale_;
  cmdpub_.publish(cmd_);
  ROS_INFO_STREAM("lin_dist:" << lin_dist << " ang_dist:" << ang_dist
                              << " lin_vel:" << cmd_.linear.x
                              << " ang_vel:" << cmd_.angular.z);

  /*
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  ac("pose_base_controller", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ac.cancelAllGoals();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped marker_pose; marker_pose.pose =
  msg->markers[0].pose; goal.target_pose = marker_pose; ac.sendGoal(goal);
*/

  visualization_msgs::Marker marker;
  geometry_msgs::Pose pose;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.id = 0;
  marker.header = msg->header;
  marker.pose = marker_pose;
  markerpub_.publish(marker);
}