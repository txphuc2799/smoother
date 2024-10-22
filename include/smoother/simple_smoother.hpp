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

#ifndef SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define SMOOTHER__SIMPLE_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include <smoother/smoother.hpp>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf2/utils.h>

namespace simple_smoother
{

/**
 * @class simple_smoother::SimpleSmoother
 * @brief A path simple_smoother implementation
 */
class SimpleSmoother : public smoother::Smoother
{
public:
  /**
   * @brief A constructor for simple_smoother::SimpleSmoother
   */
  SimpleSmoother();

  /**
   * @brief A destructor for simple_smoother::SimpleSmoother
   */
  ~SimpleSmoother();

  void initialize(ros::NodeHandle &nh, costmap_2d::Costmap2D* costmap);

  bool execSmoothPath(nav_msgs::Path &path);

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::Path & path,
    const ros::Duration & max_time);

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smoothImpl(
    nav_msgs::Path & path,
    bool & reversing_segment,
    const costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::PoseStamped & msg, const unsigned int dim,
    const double & value);

  costmap_2d::Costmap2D* costmap_;

  double tolerance_, data_w_, smooth_w_;
  double max_time_;
  int max_its_, refinement_ctr_, refinement_num_;
  bool do_refinement_;
  bool initialized_;
};

}  // namespace simple_smoother

#endif  // SMOOTHER__SIMPLE_SMOOTHER_HPP_
