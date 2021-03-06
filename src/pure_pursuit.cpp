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

#include "pure_pursuit.h"

namespace waypoint_follower
{
// Constructor
PurePursuit::PurePursuit()
  : RADIUS_MAX_(9e10)
  , KAPPA_MIN_(1 / RADIUS_MAX_)
  , is_linear_interpolation_(false)
  , is_last_waypoint(false)
  , next_waypoint_number_(-1)
  , lookahead_distance_(3)
  , lookahead_angle_(1)
  , Beta(0.9)
  , current_linear_velocity_(0)
  , pure_pursuit_mode(0)
{
}

// Destructor
PurePursuit::~PurePursuit()
{
}

double PurePursuit::calcCurvature(geometry_msgs::Pose target, double *pursuit_speed) 
{
  //*distance_data = pow(pow(getPlaneDistance(current_pose_.position, current_waypoints_.at(next_waypoint_number_).pose.pose.position), 2),0.5);//

  double kappa;
  static double speed_init = *pursuit_speed;
  static double distance_init = lookahead_distance_;
  geometry_msgs::Pose RelativePose = calcRelativePoseCoordinate(target,current_pose_);
  
  double pose_angle  = atan2(RelativePose.position.y, RelativePose.position.x);
  double orien_angle = getRelativeAngle(target, current_pose_)/2;

  double plan_angle = Beta * pose_angle + (1-Beta) * orien_angle;
  
  double distanc_ = fabs(getPlaneDistance(target.position,current_pose_.position));
  
  // modify by ybw, 2018.11.1, distance = speed
  
  *pursuit_speed = -(speed_init - 0.3)*fabs(pose_angle) + 1.4;
  lookahead_distance_ = -(distance_init - 0.5)/0.5236/2*fabs(pose_angle) + 3;// d = -2.5/(pi/6) * angle + 3
  lookahead_distance_ = lookahead_distance_<1.5? 1.5 : lookahead_distance_;
   ROS_INFO("lookahead_distance_ = %.2f", lookahead_distance_);

  //*distance_data = distanc_;
  if (distanc_ != 0)
    kappa = 2*sin(plan_angle)/distanc_;
  else
  {
    if (distanc_ > 0)
      kappa = KAPPA_MIN_;
    else
      kappa = -KAPPA_MIN_;
  }
  ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

// linear interpolation of next target
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.size());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
  geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
  // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  //double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);
  double d = getDistanceBetweenLineAndPoint(current_waypoints_.at(0).pose.pose.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise 90 degree

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  //h1.x = current_pose_.position.x + d * unit_w1.getX();
  //h1.y = current_pose_.position.y + d * unit_w1.getY();
  //h1.z = current_pose_.position.z;
  h1.x = current_waypoints_.at(0).pose.pose.position.x + d * unit_w1.getX();
  h1.y = current_waypoints_.at(0).pose.pose.position.y + d * unit_w1.getY();
  h1.z = current_waypoints_.at(0).pose.pose.position.z;

  geometry_msgs::Point h2;
  //h2.x = current_pose_.position.x + d * unit_w2.getX();
  //h2.y = current_pose_.position.y + d * unit_w2.getY();
  //h2.z = current_pose_.position.z;
  h2.x = current_waypoints_.at(0).pose.pose.position.x + d * unit_w2.getX();
  h2.y = current_waypoints_.at(0).pose.pose.position.y + d * unit_w2.getY();
  h2.z = current_waypoints_.at(0).pose.pose.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    //target1.z = current_pose_.position.z;
    target1.z = current_waypoints_.at(0).pose.pose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    //target2.z = current_pose_.position.z;
    target2.z = current_waypoints_.at(0).pose.pose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}

void PurePursuit::getNextWaypoint()
{
  int path_size = static_cast<int>(current_waypoints_.size());
  //ROS_INFO("path_size = %d",path_size);
  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    next_waypoint_number_ = -1;
    return;
  }
  if(next_waypoint_number_ == -1)
  {
    pure_follower::lane  current_path;
    current_path.waypoints = current_waypoints_;
    next_waypoint_number_ = getClosestWaypoint(current_path, current_pose_, lookahead_distance_);
    if(next_waypoint_number_ == -1)
      return;
  }

  // look for the next waypoint.
  // modify by ybw, 2018.11.2
  for (int i = next_waypoint_number_; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      if(pure_pursuit_mode==0)
      {
	      next_waypoint_number_ = i;
	      i = 0;
      }
      else if(pure_pursuit_mode==1)
      {
	      ROS_INFO("search waypoint is the last");
	      next_waypoint_number_ = i;
	      is_last_waypoint = true;
	      return;
      }
      else if(pure_pursuit_mode>1)
      {
	      next_waypoint_number_ = i;
	      i = 0;
	      pure_pursuit_mode--;
      }
    }

    // if there exists an effective waypoint
    double distance_waypointToCur = getPlaneDistance(current_waypoints_.at(i).pose.pose.position,current_pose_.position);
    bool out_lookahead_distance_ = distance_waypointToCur > lookahead_distance_;
    geometry_msgs::Pose RelativePose = calcRelativePoseCoordinate(current_waypoints_.at(i).pose.pose,current_pose_);
    bool out_lookahead_angle_ = fabs(atan2(RelativePose.position.y, RelativePose.position.x))< lookahead_angle_;
    

    //geometry_msgs::Point _way_point = current_waypoints_.at(i).pose.pose.position;
    //geometry_msgs::Point _cur_point = current_pose_.position;
    //double _angle = fabs(atan2(RelativePose.position.y, RelativePose.position.x));
    //double _distance = getPlaneDistance(current_waypoints_.at(i).pose.pose.position,current_pose_.position);
    //ROS_INFO("_way_point x=%.3f,y=%.3f",_way_point.x,_way_point.y);
    //ROS_INFO("_cur_point x=%.3f,y=%.3f",_cur_point.x,_cur_point.y);
    //ROS_INFO("i=%d,_angle=%.3f,_distance=%.3f",i,_angle,_distance);
    

    if (out_lookahead_distance_ && out_lookahead_angle_)
    {
      ROS_INFO("path_size = %d,next_waypoint_number_=%d",path_size,next_waypoint_number_);
      next_waypoint_number_ = i;
      return;
    }
    else if(distance_waypointToCur > 10*lookahead_distance_)
    {
	    ROS_INFO("distance_waypointToCur=%.3f,lookahead_distance_=%.3f",distance_waypointToCur,lookahead_distance_);
      break;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number_ = -1;
  return;
}

bool PurePursuit::canGetCurvature(double *output_kappa, double *pursuit_speed)
{
  // search next waypoint
  getNextWaypoint();
  if (next_waypoint_number_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }

  next_target_position_ = current_waypoints_.at(next_waypoint_number_).pose.pose;
  *output_kappa = calcCurvature(next_target_position_, pursuit_speed);

  return (!is_last_waypoint);
}

}  // waypoint_follower
