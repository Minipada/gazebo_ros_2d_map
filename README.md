# gazebo_ros_2Dmap_plugin
Gazebo simulator plugin to automatically generate a 2D occupancy map from the simulated world at a given certain height.

This plugin was adapted from the [octomap plugin](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_gazebo_plugins) from ETH Zürich.

Include this in your world file:

```xml
<plugin name='gazebo_occupancy_map' filename='libgazebo_ros_2d_map.so'>
      <map_resolution>0.05</map_resolution>
      <map_height>0</map_height>
      <map_size_x>55</map_size_x>
      <map_size_y>55</map_size_y>
      <init_robot_x>-7.3</init_robot_x>
      <init_robot_y>1.37</init_robot_y>
</plugin>
```

To generate the map, call the `/gazebo_2Dmap_plugin/generate_map` ros service:

```bash
ros2 service call /gazebo_2Dmap_plugin/generate_map
```

The generated map is published on the `/map2d` ros topic.

You can use the `map_saver_cli` node from the `nav2_map_server` package inside ros navigation to save your generated map to a .pgm and .yaml file:

```bash
ros2 run nav2_map_server map_saver_cli -f <mapname> /map:=/map2d
```
The last map generated with the ```/gazebo_2Dmap_plugin/generate_map``` call is saved.

## Hints

* To identify the connected free space the robot would discover during mapping, the plugin performs a wavefront exploration along the occupancy grid starting from the origin of the gazebo world coordinate system. Please ensure that the corresponding cell is in the continuous free space.
* The plugin will map all objects in the world, including the robot. Remove all unwanted  objects before creating the map.
