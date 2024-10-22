#ifndef SMOOTHER__SMOOTHER_HPP_
#define SMOOTHER__SMOOTHER_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>

namespace smoother
{
class Smoother
{
public:
    virtual void initialize(ros::NodeHandle &nh, costmap_2d::Costmap2D* costmap) = 0;
    virtual bool execSmoothPath(nav_msgs::Path &path) = 0;
    virtual ~Smoother(){}

protected:
    Smoother(){}
};
} // namespace smoother

#endif // SMOOTHER__SMOOTHER_HPP_