/*
 * RobotinoLocalPlanner.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#include "robotino_local_planner/RobotinoLocalPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

PLUGINLIB_DECLARE_CLASS(robotino_local_planner, RobotinoLocalPlanner, robotino_local_planner::RobotinoLocalPlanner, nav_core::BaseLocalPlanner)

#define TRANSFORM_TIMEOUT 0.5
#define PI 3.141592653

namespace robotino_local_planner
{
  RobotinoLocalPlanner::RobotinoLocalPlanner():
      tf_(NULL),
      state_(Finished),
      curr_heading_index_(0),
      next_heading_index_(0),
      max_linear_vel_(0.0),
      min_linear_vel_(0.0),
      max_rotation_vel_(0.0),
      min_rotation_vel_(0.0),
      num_window_points_(10),
      tf_stream_(tf_stream_nh_),
      tf_stream_handle_(tf_stream_.addTransform("map", "base_link", boost::bind(&RobotinoLocalPlanner::poseCb, this, _1)))
  {
  }

  RobotinoLocalPlanner::~RobotinoLocalPlanner()
  {
    // Empty
  }

  void RobotinoLocalPlanner::poseCb(const geometry_msgs::TransformStampedConstPtr& transform)
  {
    boost::mutex::scoped_lock lock(pose_lock_);
    last_pose_ = transform->transform;
  }

  void RobotinoLocalPlanner::initialize( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros )
  {
    tf_ = tf;

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("heading_lookahead", heading_lookahead_, 0.3 );
    private_nh.param("max_linear_vel", max_linear_vel_, 0.3 );
    private_nh.param("min_linear_vel", min_linear_vel_, 0.1 );
    private_nh.param("max_rotation_vel", max_rotation_vel_, 1.0 );
    private_nh.param("min_rotation_vel", min_rotation_vel_, 0.3 );
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05 );
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10 );
    private_nh.param("num_window_points", num_window_points_, 10 );

    next_heading_pub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 10);

    ROS_INFO("RobotinoLocalPlanner initialized");
  }

  bool RobotinoLocalPlanner::computeVelocityCommands( geometry_msgs::Twist& cmd_vel)
  {
    boost::mutex::scoped_lock lock(pose_lock_);

    // Set all values of cmd_vel to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // Set the default return value as false
    bool ret = false;

    // We need to compute the next heading point from the global plan
    computeNextHeadingIndex();
    publishNextHeading();

    switch(state_)
    {
    case RotatingToStart:
      ret = rotateToStart( cmd_vel );
      break;
    case Moving:
      ret = move( cmd_vel );
      break;
    case RotatingToGoal:
      ret = rotateToGoal( cmd_vel );
      break;
    default:
      return true;
    }

    return ret;
  }

  bool RobotinoLocalPlanner::isGoalReached()
  {
    return ( state_ == Finished );
  }

  bool RobotinoLocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan )
  {
    global_plan_.clear();

    // Make our copy of the global plan
    global_plan_ = global_plan;

    ROS_INFO_STREAM("Global plan size: " << global_plan_.size() );

    curr_heading_index_ = 0;
    next_heading_index_ = 0;

    // Set the state to RotatingToStart
    state_ = RotatingToStart;
    return true;
  }

  void RobotinoLocalPlanner::publishNextHeading(bool show )
  {
    const geometry_msgs::PoseStamped& next_pose = global_plan_[next_heading_index_];

    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id= next_pose.header.frame_id;
    marker.ns = "waypoints";
    marker.type = visualization_msgs::Marker::CYLINDER;

    if(show)
    {
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.pose = next_pose.pose;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.2;
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    next_heading_pub_.publish(marker);
  }

  void RobotinoLocalPlanner::displacementToGoal(const geometry_msgs::PoseStamped& goal, double& x, double& y, double& rotation) const
  {
    // Create a vector between the current pose to the next heading pose
    x = goal.pose.position.x - last_pose_.translation.x;
    y = goal.pose.position.y - last_pose_.translation.y;

    // Calculate the rotation between the current and the vector created above
    rotation = angles::shortest_angular_distance(tf::getYaw(last_pose_.rotation), std::atan2(y, x));
  }

  bool RobotinoLocalPlanner::rotateToStart( geometry_msgs::Twist& cmd_vel )
  {
    ros::Time now = ros::Time::now();
    global_plan_[next_heading_index_].header.stamp = now;

    double x, y, rotation;
    displacementToGoal(global_plan_[next_heading_index_], x, y, rotation);

    if( fabs( rotation ) < yaw_goal_tolerance_ )
    {
      state_ = Moving;
      return true;
    }

    cmd_vel.angular.z = calRotationVel( rotation );

    return true;
  }

  bool RobotinoLocalPlanner::move( geometry_msgs::Twist& cmd_vel )
  {
    ros::Time now = ros::Time::now();
    global_plan_[next_heading_index_].header.stamp = now;

    double x, y, rotation;
    displacementToGoal(global_plan_[next_heading_index_], x, y, rotation);

    cmd_vel.angular.z = calRotationVel( rotation );

    if( fabs( rotation ) < yaw_goal_tolerance_ )
    {
      // The robot has rotated to its next heading pose
      cmd_vel.angular.z = 0.0;
    }

    cmd_vel.linear.x = calLinearVel();

    // We are approaching the goal position, slow down
    if( next_heading_index_ == (int) global_plan_.size()-1)
    {
      const double distance_to_next_heading = linearDistance(last_pose_.translation, global_plan_[next_heading_index_].pose.position);

      // Reached the goal, now we can stop and rotate the robot to the goal position
      if (distance_to_next_heading < xy_goal_tolerance_)
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        state_ = RotatingToGoal;
        return true;
      }
    }

    return true;
  }

  bool RobotinoLocalPlanner::rotateToGoal( geometry_msgs::Twist& cmd_vel )
  {
    ros::Time now = ros::Time::now();
    global_plan_[next_heading_index_].header.stamp = now;

    double rotation = tf::getYaw( global_plan_[next_heading_index_].pose.orientation ) -
      tf::getYaw(last_pose_.rotation);

    if( fabs( rotation ) < yaw_goal_tolerance_ )
    {
      state_ = Finished;
      ROS_INFO("Goal reached");
      return true;
    }

    cmd_vel.angular.z = calRotationVel( rotation );

    return true;
  }

  void RobotinoLocalPlanner::computeNextHeadingIndex()
  {
    geometry_msgs::PoseStamped next_heading_pose;

    for( unsigned int i = curr_heading_index_; i < global_plan_.size() - 1; ++i )
    {
      ros::Time now = ros::Time::now();
      global_plan_[i].header.stamp = now;

      next_heading_pose = global_plan_[i];

      double dist = linearDistance( last_pose_.translation,
          next_heading_pose.pose.position );

      if( dist > heading_lookahead_)
      {
        next_heading_index_ = i;
        return;
      }
      else
      {
        curr_heading_index_++;
      }
    }
    next_heading_index_ = global_plan_.size() - 1;
  }

  double RobotinoLocalPlanner::calLinearVel()
  {
    double vel = 0.0;

    unsigned int beg_index;
    if (next_heading_index_ < num_window_points_)
    {
      beg_index = 0;
    }
    else
    {
      beg_index = next_heading_index_ - num_window_points_;
    }

    double straight_dist = linearDistance(global_plan_[beg_index].pose.position,
        global_plan_[next_heading_index_].pose.position);

    double path_dist = 0.0;

    for(unsigned int i = beg_index; i < (unsigned int)next_heading_index_; ++i)
    {
      path_dist = path_dist + linearDistance(global_plan_[i].pose.position,
              global_plan_[i + 1].pose.position);
    }

    const double diff = std::max(0.001, path_dist - straight_dist);

    vel = 0.001 * ( 1 / diff );

    if( vel > max_linear_vel_ )
      vel = max_linear_vel_;

    if( vel < min_linear_vel_ )
      vel = min_linear_vel_;

    return vel;
  }

  double RobotinoLocalPlanner::calRotationVel( double rotation )
  {
    double vel = 0.0;

    if( fabs(rotation) > max_rotation_vel_)
    {
      vel = std::copysign(max_rotation_vel_, rotation);
    }
    else if ( fabs(rotation) < min_rotation_vel_ )
    {
      vel = std::copysign(min_rotation_vel_, rotation);
    }
    else
    {
      vel = rotation;
    }

    return vel;
  }

  double RobotinoLocalPlanner::linearDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 )
  {
    return hypot(p2.x - p1.x, p2.y - p1.y);
  }

  double RobotinoLocalPlanner::linearDistance( geometry_msgs::Vector3 t, geometry_msgs::Point p )
  {
    return hypot(t.x - p.x, t.y - p.y);
  }

}
