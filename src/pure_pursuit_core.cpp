/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "pure_pursuit_core.h"
#include <std_msgs/Float64.h> 
#include <stdio.h>

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , is_velocity_set_(false)
  , is_config_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(1.4)
  , param_flag_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(5.0)
  , lookahead_distance_ratio_(2.0)
  , minimum_lookahead_distance_(6.0)
{
  initForROS();
  readGoaTxt();
  // initialize for PurePursuit  是否进行线性插值
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
  // ROS_INFO_STREAM("is_linear_interpolation : " << is_linear_interpolation_);
  private_nh_.param("publishes_for_steering_robot", publishes_for_steering_robot_, bool(false));
  private_nh_.param("vehicle_info/wheel_base", wheel_base_, double(2.7));

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10, &PurePursuitNode::callbackFromWayPoints, this);/*Path point planned by the path planner*/
  sub2_ = nh_.subscribe("/localization_integrated_estimate", 10, &PurePursuitNode::callbackFromCurrentPose, this);/*Robot published pos*/
  sub3_ = nh_.subscribe("config/waypoint_follower", 10, &PurePursuitNode::callbackFromConfig, this);/*Robot parameter data*/
  sub4_ = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);/*Robot current speed*/
  sub5_ = nh_.subscribe("/odom", 10, &PurePursuitNode::callbackFromOdomPose, this);/*Robot published pos*/

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::Twist>("cmdVelocity", 10);/*Publish robot motion information*/
  pub2_ = nh_.advertise<pure_follower::ControlCommandStamped>("ctrl_cmd", 10);/*publish robot control information*/
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("current_point_marker", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  pub15_ = nh_.advertise<visualization_msgs::Marker>("rajectory_circle_mark", 0);
  pub17_ = nh_.advertise<std_msgs::Float64>("distance_to_next_waypoint", 0.0);
  pub18_ = nh_.advertise<visualization_msgs::Marker>("Path_waypoint_mark", 10);
}

void PurePursuitNode::readGoaTxt()
{
  Point3D fitPoint[10240];
  float uk[10240];
  float U[10240];
  int count = 0;
  int num_skip = 10;
  int i = 0;
  float u = 0.0;
  
  std::fstream inStream;
  inStream.open("goal.txt",std::ios::in);
  if(!inStream.is_open())
  {
    ROS_INFO("open goal.txt false!!!");  
    return ;     
  }

  geometry_msgs::Point point_msg;
  geometry_msgs::Point theta_msg;

  while(!inStream.eof())
  {
   inStream>>point_msg.x>>point_msg.y>>point_msg.z>>theta_msg.x>>theta_msg.y>>theta_msg.z;
   i++;
   if(i>=num_skip)
   {
    fitPoint[count].X = point_msg.x;
    fitPoint[count].Y = point_msg.y;
    fitPoint[count].Z = point_msg.z;
	// ROS_INFO("fitPoint x=%f y=%f",point_msg.x,point_msg.y);
    count++;
    i=0;
   }
  }
  //最后一个点
  fitPoint[count].X = point_msg.x;
  fitPoint[count].Y = point_msg.y;
  fitPoint[count].Z = point_msg.z;
  count++;
   
  ROS_INFO("fitPoint count=%d",count);
  getFitPointU(fitPoint,count,0,uk);
  getVectorU(count,uk,U);
  
  std::vector<pure_follower::waypoint> current_waypoints_;
  pure_follower::waypoint _wayPose;
  while(u<1-0.0001)
  {
    Point3D _wayPoint = getCurvePoint(u,count, U, fitPoint);
    Point3D _wayPointNext = getCurvePoint(u+0.0001,count, U, fitPoint);

    float yaw =atan2(_wayPointNext.Y - _wayPoint.Y, _wayPointNext.X - _wayPoint.X);
    
    _wayPose.pose.pose.position.x = _wayPoint.X;
    _wayPose.pose.pose.position.y = _wayPoint.Y;
    _wayPose.pose.pose.position.z = _wayPoint.Z; //fitPoint[(int)(count * u)].Z;
    ROS_INFO("_wayPoint.Z=%f",_wayPoint.Z);
    _wayPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      
    current_waypoints_.push_back(_wayPose);
    u += 0.0002;  
  }   
  pp_.setCurrentWaypoints(current_waypoints_);
  
  is_waypoint_set_ = true;
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();

    //waitdata
    
    if (!is_pose_set_ || !is_waypoint_set_ ) //|| !is_velocity_set_ || !is_config_set_
    {
     
      ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }
    
    double kappa = 0;
    
    bool can_get_curvature = pp_.canGetCurvature(&kappa,&command_linear_velocity_);
    
    publishTwistStamped(can_get_curvature, kappa);// v ,w
    publishControlCommandStamped(can_get_curvature, kappa);// v,theta

    std_msgs::Float64 distance;
    
    // modify by ybw, 2018.11.2
    //distance.data = distance_data;
    
    pub17_.publish(distance);

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayCurrentpoint(pp_.getCurrentPose().position));
    pub15_.publish(displayTrajectoryCircle( waypoint_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget().position, pp_.getCurrentPose())));
    pub18_.publish(displayPathWaypoint(pp_.getCurrentWaypoints()));
    
   // is_pose_set_ = false;
   // is_velocity_set_ = false;
   // is_waypoint_set_ = false;
    loop_rate.sleep();
  }
}

void PurePursuitNode::publishTwistStamped(const bool &can_get_curvature, const double &kappa) const
{
  geometry_msgs::Twist twist;
  //ts.header.stamp = ros::Time::now();
  twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  twist.angular.z = can_get_curvature ? kappa * twist.linear.x : 0;//w=v/r
  pub1_.publish(twist);
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &kappa) const
{
  if (!publishes_for_steering_robot_)
    return;

  pure_follower::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.steering_angle = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}

double PurePursuitNode::computeLookaheadDistance() const
{
  if (param_flag_ == enumToInteger(Mode::dialog))
    return const_lookahead_distance_;

  double maximum_lookahead_distance = current_linear_velocity_ * 10;
  double ld = current_linear_velocity_ * lookahead_distance_ratio_;//ld=k*v

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_
        : ld > maximum_lookahead_distance ? maximum_lookahead_distance
        : ld;
}

double PurePursuitNode::computeCommandVelocity() const
{
  if (param_flag_ == enumToInteger(Mode::dialog))
    return kmph2mps(const_velocity_);

  return command_linear_velocity_;
}

void PurePursuitNode::callbackFromConfig(const pure_follower::ConfigWaypointFollowerConstPtr &config)
{
  param_flag_ = config->param_flag;
  const_lookahead_distance_ = config->lookahead_distance;
  const_velocity_ = config->velocity;
  lookahead_distance_ratio_ = 1.0;
  minimum_lookahead_distance_ = 1.0;
  is_config_set_ = true;
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg)
{
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromOdomPose(const nav_msgs::Odometry &amcl_pose)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose=amcl_pose.pose.pose;
  pp_.setCurrentPose(pose_stamped);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_linear_velocity_ = msg->twist.linear.x;
  pp_.setCurrentVelocity(current_linear_velocity_);
  is_velocity_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const pure_follower::laneConstPtr &msg)
{
  if (!msg->waypoints.empty())
    command_linear_velocity_ = msg->waypoints.at(0).twist.twist.linear.x;
  else
    command_linear_velocity_ = 0;

  pp_.setCurrentWaypoints(msg->waypoints);
  is_waypoint_set_ = true;
}

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
	return atan(wheel_base * kappa);/*theta = arctan(k*l)*/
}

}  // waypoint_follower
