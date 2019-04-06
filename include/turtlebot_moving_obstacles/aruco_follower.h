#pragma once

#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


//* The aruco follower
/**
 * This class subscribes to an aruco marker pose and publishes command vel
 * messages
 */
class ArucoFollower 
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  ArucoFollower() : lin_vel_scale_(0.0), ang_vel_scale_(0.0)
  {

  }

  ~ArucoFollower()
  {

  }

  /*!
   * @brief initialize method from node handle.
   * Sets up the parameters and topics.
   */
  void initialize(ros::NodeHandle& n1, ros::NodeHandle& n2);


  /**
    * Method to update current pose
    * @return bool success or failure in transform
    */
  bool updatePose();

private:
	static const int DIST_FROM_TARGET;
	geometry_msgs::PoseStamped robot_pose_;
  double lin_vel_scale_; /**< The scaling factor for translational robot speed */
  double ang_vel_scale_; /**< The scaling factor for rotational robot speed */
  tf::TransformListener transform_listener_;   /**< listener for transform conversions */
  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  geometry_msgs::Twist cmd_;

  /*!
   * @brief Callback for detected marker.
   * Generates command vel based on marker position
   * @param msg MarkerDetction msg
   */
  void markercb(const marker_msgs::MarkerDetectionConstPtr& msg);

    /**
   * Angular difference between two points
   * @param val second vector
   * @return double angular difference
   */
  double getAngle(geometry_msgs::Point& p1, geometry_msgs::Point& p2) { return atan2((p2.y - p1.y), (p2.x - p1.x)); }

  /**
   * Eucledian distance between two points
   * @param val second vector
   * @return double distance
   */
  double getDistance(geometry_msgs::Point& p1, geometry_msgs::Point& p2) {
    return sqrt((pow(p2.y - p1.y, 2.0)) + (pow(p2.x - p1.x, 2.0)));
  }


};
