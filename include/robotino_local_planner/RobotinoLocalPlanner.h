/*
 * RobotinoLocalPlanner.h
 *
 *  Created on: Feb 20, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOLOCALPLANNER_H_
#define ROBOTINOLOCALPLANNER_H_

#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf_lookup/tf_stream_client.h>
#include <tf_lookup/tf_stream_client_impl.h>

namespace robotino_local_planner
{
  class RobotinoLocalPlanner: public nav_core::BaseLocalPlanner
  {
  public:
    RobotinoLocalPlanner();
    ~RobotinoLocalPlanner();

    void initialize( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros );
    bool computeVelocityCommands( geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan );

  private:
    void poseCb(const geometry_msgs::TransformStampedConstPtr& transform);

    void publishNextHeading(bool show = true);
    void displacementToGoal(const geometry_msgs::PoseStamped& goal, double& x, double& y, double& rotation) const;
    bool rotateToStart( geometry_msgs::Twist& cmd_vel );
    bool move( geometry_msgs::Twist& cmd_vel );
    bool rotateToGoal( geometry_msgs::Twist& cmd_vel );
    void computeNextHeadingIndex();
    double calLinearVel();
    double calRotationVel( double rotation );
    double linearDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 );
    double linearDistance( geometry_msgs::Vector3 t, geometry_msgs::Point p );
    double mapToMinusPIToPI( double angle ) const;

    typedef enum { RotatingToStart, Moving, RotatingToGoal, Finished } State;

    tf::TransformListener* tf_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    ros::Publisher next_heading_pub_;

    State state_;

    int curr_heading_index_, next_heading_index_;

    // Parameters
    double heading_lookahead_;
    double max_linear_vel_, min_linear_vel_;
    double max_rotation_vel_, min_rotation_vel_;
    double yaw_goal_tolerance_, xy_goal_tolerance_;
    int num_window_points_;

    ros::NodeHandle tf_stream_nh_;
    tf_lookup::TfStreamClient tf_stream_;
    tf_lookup::TfStreamClient::Handle tf_stream_handle_;

    boost::mutex pose_lock_;
    geometry_msgs::Transform last_pose_;

  };
}

#endif /* ROBOTINOLOCALPLANNER_H_ */
