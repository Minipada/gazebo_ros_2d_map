#ifndef GAZEBO_2DMAP_PLUGIN_H
#define GAZEBO_2DMAP_PLUGIN_H

#include <chrono>
#include <functional>
#include <iostream>
#include <math.h>
#include <memory>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <octomap/octomap.h>
#include <sdf/sdf.hh>

#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "gazebo_ros_2d_map/common.h"

using namespace std::chrono_literals;

namespace gazebo_ros_2d_map
{

/// \brief    Octomap plugin for Gazebo.
/// \details  This plugin is dependent on ROS, and is not built if NO_ROS=TRUE is provided to
///           CMakeLists.txt. The PX4/Firmware build does not build this file.
class OccupancyMapFromWorld : public gazebo::WorldPlugin
{
public:
  OccupancyMapFromWorld();
  virtual ~OccupancyMapFromWorld();

protected:
  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  bool worldCellIntersection(const ignition::math::Vector3d& cell_center, const double cell_length,
                             gazebo::physics::RayShapePtr ray);

  /*! \brief
   */
  void createOccupancyMap();

  static void cell2world(unsigned int cell_x, unsigned int cell_y, double map_size_x, double map_size_y,
                         double map_resolution, double& world_x, double& world_y);

  static void world2cell(double world_x, double world_y, double map_size_x, double map_size_y, double map_resolution,
                         unsigned int& cell_x, unsigned int& cell_y);

  static bool cell2index(unsigned int cell_x, unsigned int cell_y, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index);

  static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y, unsigned int& cell_x,
                         unsigned int& cell_y);
  void timerCallback();

private:
  bool createMapServiceCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                std_srvs::srv::Empty::Response::SharedPtr response);

  gazebo::physics::WorldPtr world_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr map_service_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  nav_msgs::msg::OccupancyGrid* occupancy_map_ = NULL;
  double map_resolution_;
  double map_height_;
  double map_size_x_;
  double map_size_y_;
  double init_robot_x_;
  double init_robot_y_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gazebo_ros_2d_map

#endif  // GAZEBO_2DMAP_PLUGIN_H
