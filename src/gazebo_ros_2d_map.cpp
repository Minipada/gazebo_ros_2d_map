#include "gazebo_ros_2d_map/gazebo_ros_2d_map.hpp"

namespace gazebo_ros_2d_map
{

OccupancyMapFromWorld::~OccupancyMapFromWorld()
{
}

OccupancyMapFromWorld::OccupancyMapFromWorld() : WorldPlugin()
{
}

void OccupancyMapFromWorld::Load(gazebo::physics::WorldPtr parent, sdf::ElementPtr sdf)
{
  RCLCPP_INFO(rclcpp::get_logger("gazebo_ros2_2d_map"), "Loading gazebo_ros2_2d_map plugin");
  if (gazebo::kPrintOnPluginLoad)
  {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  world_ = parent;

  map_resolution_ = 0.05;

  if (sdf->HasElement("map_resolution"))
    map_resolution_ = sdf->GetElement("map_resolution")->Get<double>();

  map_height_ = 0.3;

  if (sdf->HasElement("map_z"))
    map_height_ = sdf->GetElement("map_z")->Get<double>();

  init_robot_x_ = 0.0;

  if (sdf->HasElement("init_robot_x"))
    init_robot_x_ = sdf->GetElement("init_robot_x")->Get<double>();

  init_robot_y_ = 0.0;

  if (sdf->HasElement("init_robot_y"))
    init_robot_y_ = sdf->GetElement("init_robot_y")->Get<double>();

  map_size_x_ = 10.0;

  if (sdf->HasElement("map_size_x"))
    map_size_x_ = sdf->GetElement("map_size_x")->Get<double>();

  map_size_y_ = 10.0;

  if (sdf->HasElement("map_size_y"))
    map_size_y_ = sdf->GetElement("map_size_y")->Get<double>();

  if (map_size_x_ != map_size_y_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("gazebo_ros2_2d_map_plugin"),
                 "map_size_x must be equal to map_size_y. The map generated must be a square, reconfigure your "
                 "plugin. Current setting: x=%lf y=%lf",
                 map_size_x_, map_size_y_);
    return;
  }

  node_ = gazebo_ros::Node::Get(sdf);  // rclcpp::Node::SharedPtr("gazebo_2d_map_plugin");
  rclcpp::QoS map_qos(10);             // initialize to default
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);
  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_from_gazebo", map_qos);

  map_service_ = node_->create_service<std_srvs::srv::Empty>(
      "gazebo_2d_map_plugin/generate_map",
      std::bind(&OccupancyMapFromWorld::createMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  timer_ = node_->create_wall_timer(2s, std::bind(&OccupancyMapFromWorld::timerCallback, this));
}

void OccupancyMapFromWorld::timerCallback()
{
  if (occupancy_map_ != NULL)
  {
    map_pub_->publish(*occupancy_map_);
  }
}

bool OccupancyMapFromWorld::createMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                     std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  createOccupancyMap();
  return true;
}

bool OccupancyMapFromWorld::worldCellIntersection(const ignition::math::Vector3d& cell_center, const double cell_length,
                                                  gazebo::physics::RayShapePtr ray)
{
  // check for collisions with rays surrounding the cell
  //     ---
  //    | + |
  //     ---

  double dist;
  std::string entity_name;

  int cell_length_steps = 1;
  double side_length;

  // check for collisions with beams at increasing sizes to capture smaller
  // objects inside the cell
  for (int step = 1; step <= cell_length_steps; step++)
  {
    side_length = cell_length * step / cell_length_steps;

    for (int i = -1; i < 2; i += 2)
    {
      double start_x = cell_center.X() + i * side_length / 2;
      double start_y = cell_center.Y() - i * side_length / 2;

      for (int j = -1; j < 2; j += 2)
      {
        double end_x = cell_center.X() + j * side_length / 2;
        double end_y = cell_center.Y() + j * side_length / 2;

        ray->SetPoints(ignition::math::Vector3d(start_x, start_y, cell_center.Z()),
                       ignition::math::Vector3d(end_x, end_y, cell_center.Z()));
        ray->GetIntersection(dist, entity_name);

        if (!entity_name.empty())
          return true;
      }
    }
  }

  return false;
}

void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y, double map_size_x, double map_size_y,
                                       double map_resolution, double& world_x, double& world_y)
{
  world_x = cell_x * map_resolution - map_size_x / 2;
  world_y = cell_y * map_resolution - map_size_y / 2;
}

void OccupancyMapFromWorld::world2cell(double world_x, double world_y, double map_size_x, double map_size_y,
                                       double map_resolution, unsigned int& cell_x, unsigned int& cell_y)
{
  cell_x = std::round((world_x + map_size_x / 2) / map_resolution);
  cell_y = std::round((world_y + map_size_y / 2) / map_resolution);
}

bool OccupancyMapFromWorld::cell2index(unsigned int cell_x, unsigned int cell_y, unsigned int cell_size_x,
                                       unsigned int cell_size_y, unsigned int& map_index)
{
  if (cell_x < cell_size_x && cell_y < cell_size_y)
  {
    map_index = cell_y * cell_size_y + cell_x;
    return true;
  }
  else
  {
    // return false when outside map bounds
    return false;
  }
}

bool OccupancyMapFromWorld::index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_y = index / cell_size_y;
  cell_x = index % cell_size_x;

  if (cell_x < cell_size_x && cell_y < cell_size_y)
    return true;
  else
  {
    // return false when outside map bounds
    return false;
  }
}

void OccupancyMapFromWorld::createOccupancyMap()
{
  ignition::math::Vector3d map_origin(init_robot_x_, init_robot_y_, map_height_);

  unsigned int cells_size_x = map_size_x_ / map_resolution_;
  unsigned int cells_size_y = map_size_y_ / map_resolution_;

  occupancy_map_ = new nav_msgs::msg::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);
  // all cells are initially unknown
  std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
  occupancy_map_->header.stamp = rclcpp::Clock().now();
  occupancy_map_->header.frame_id = "odom";
  occupancy_map_->info.map_load_time = rclcpp::Time(0);
  occupancy_map_->info.resolution = map_resolution_;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
  occupancy_map_->info.origin.position.x = map_origin.X() - map_size_x_ / 2;
  occupancy_map_->info.origin.position.y = map_origin.Y() - map_size_y_ / 2;
  occupancy_map_->info.origin.position.z = map_origin.Z();
  occupancy_map_->info.origin.orientation.w = 1;

  gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Starting wavefront expansion for mapping" << std::endl;

  // identify free space by spreading out from initial robot cell
  double robot_x = init_robot_x_;
  double robot_y = init_robot_y_;

  // find initial robot cell
  unsigned int cell_x, cell_y, map_index;
  world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_, cell_x, cell_y);

  if (!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
  {
    RCLCPP_ERROR(rclcpp::get_logger("gazebo_ros2_2d_map_plugin"),
                 "initial robot pos is outside map, could not create map.");
    return;
  }

  std::vector<unsigned int> wavefront;
  wavefront.push_back(map_index);

  // wavefront expansion for identifying free, unknown and occupied cells
  while (!wavefront.empty())
  {
    map_index = wavefront.at(0);
    wavefront.erase(wavefront.begin());

    index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

    // mark cell as free
    occupancy_map_->data.at(map_index) = 0;

    // explore cells neighbors in an 8-connected grid
    unsigned int child_index;
    double world_x, world_y;
    uint8_t child_val;

    // 8-connected grid
    for (int i = -1; i < 2; i++)
    {
      for (int j = -1; j < 2; j++)
      {
        // makes sure index is inside map bounds
        if (cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
        {
          child_val = occupancy_map_->data.at(child_index);

          // only update value if cell is unknown
          if (child_val != 100 && child_val != 0 && child_val != 50)
          {
            cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_, world_x, world_y);

            bool cell_occupied =
                worldCellIntersection(ignition::math::Vector3d(world_x, world_y, map_height_), map_resolution_, ray);

            if (cell_occupied)
            {
              // mark cell as occupied
              occupancy_map_->data.at(child_index) = 100;
            }
            else
            {
              // add cell to wavefront
              wavefront.push_back(child_index);
              // mark wavefront in map so we don't add children to wavefront multiple
              // times
              occupancy_map_->data.at(child_index) = 50;
            }
          }
        }
      }
    }
    map_pub_->publish(*occupancy_map_);
  }

  std::cout << "\rOccupancy Map generation completed" << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo_ros_2d_map
