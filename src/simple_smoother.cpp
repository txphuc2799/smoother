// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <vector>
#include <memory>
#include <smoother/simple_smoother.hpp>
#include <smoother/smoother_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(simple_smoother::SimpleSmoother, smoother::Smoother)

namespace simple_smoother
{
using namespace std::chrono;  // NOLINT

SimpleSmoother::SimpleSmoother()
: costmap_(nullptr), initialized_(false)
{
}

SimpleSmoother::~SimpleSmoother()
{
}

void SimpleSmoother::initialize(ros::NodeHandle &nh, costmap_2d::Costmap2D* costmap)
{
  if (!initialized_) {
    costmap_ = costmap;

    nh.param("SimpleSmoother/max_time", max_time_, 5.0);
    nh.param("SimpleSmoother/tolerance", tolerance_, 1e-10);
    nh.param("SimpleSmoother/max_its", max_its_, 1000);
    nh.param("SimpleSmoother/w_data", data_w_, 0.2);
    nh.param("SimpleSmoother/w_smooth", smooth_w_, 0.3);
    nh.param("SimpleSmoother/do_refinement", do_refinement_, true);
    nh.param("SimpleSmoother/refinement_num", refinement_num_, 2);
  }
}

bool SimpleSmoother::execSmoothPath(nav_msgs::Path &path)
{
  if (!smooth(path, ros::Duration(max_time_))) {
    return false;
  }
  return true;
}

bool SimpleSmoother::smooth(
  nav_msgs::Path & path,
  const ros::Duration & max_time)
{
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.toSec();

  bool success = true, reversing_segment;
  unsigned int segments_smoothed = 0;
  nav_msgs::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<smoother_utils::PathSegment> path_segments = smoother_utils::findDirectionalPathSegments(path);

  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 9) {
      // Populate path segment
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.toSec() - duration_cast<duration<double>>(now - start).count();
      refinement_ctr_ = 0;

      bool segment_was_smoothed = smoothImpl(
        curr_path_segment, reversing_segment, costmap_, time_remaining);

      if (segment_was_smoothed) {
        segments_smoothed++;
      }

      // Smooth path segment naively
      success = success && segment_was_smoothed;

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool SimpleSmoother::smoothImpl(
  nav_msgs::Path & path,
  bool & reversing_segment,
  const costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  steady_clock::time_point a = steady_clock::now();
  ros::Duration max_dur = ros::Duration(max_time);

  int its = 0;
  double change = tolerance_;
  const unsigned int & path_size = path.poses.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  unsigned int mx, my;

  nav_msgs::Path new_path = path;
  nav_msgs::Path last_path = path;

  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    if (its >= max_its_) {
      ROS_WARN(
        "Number of iterations has exceeded limit of %i.", max_its_);
      path = last_path;
      smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    // Make sure still have time left to process
    steady_clock::time_point b = steady_clock::now();
    duration<double> timespan = b - a;
    ros::Duration ros_timespan(timespan.count());
    if (ros_timespan > max_dur) {
      ROS_WARN(
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);
      path = last_path;
      smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
      ROS_ERROR("Smoothing time exceed allowed duration");
      return false;
    }

    for (unsigned int i = 1; i != path_size - 1; i++) {
      for (unsigned int j = 0; j != 2; j++) {
        x_i = getFieldByDim(path.poses[i], j);
        y_i = getFieldByDim(new_path.poses[i], j);
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
        y_i_org = y_i;

        // Smooth based on local 3 point neighborhood and original data locations
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        setFieldByDim(new_path.poses[i], j, y_i);
        change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      float cost = 0.0;
      if (costmap) {
        costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);
        cost = static_cast<float>(costmap->getCost(mx, my));
      }
      static constexpr unsigned char MAX_NON_OBSTACLE = 252;
      if (cost > MAX_NON_OBSTACLE && cost != costmap_2d::NO_INFORMATION) {
        ROS_DEBUG(
          "SmacPlannerSmoother: "
          "Smoothing process resulted in an infeasible collision. "
          "Returning the last path before the infeasibility was introduced.");
        path = last_path;
        smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
        return false;
      }
    }

    last_path = new_path;
  }

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  // but really puts the path quality over the top.
  if (do_refinement_ && refinement_ctr_ < refinement_num_) {
    refinement_ctr_++;
    smoothImpl(new_path, reversing_segment, costmap, max_time);
  }

  smoother_utils::updateApproximatePathOrientations(new_path, reversing_segment);
  path = new_path;
  return true;
}

double SimpleSmoother::getFieldByDim(
  const geometry_msgs::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void SimpleSmoother::setFieldByDim(
  geometry_msgs::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

}  // namespace smoother
