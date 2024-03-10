## Cartographer-ROS Installation 

https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
Create a seperate workspace for Cartographer and build it using catkin_make_isolated

## Create a package in catkin_ws/src for SLAM.
cd ~/catkin_ws/src/SLAM
mdkir launch
cd launch/
cp install_isolated/share/cartographer_ros/launch/demo_backpack_2d.launch my_robot.launch

--- And create a lua folder in SLAM
mkdir lua
cd lua/
cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua my_robot.lua

## Integrate with your cartographer in package.xml according to the added file.

## And build the src folder using catkin build and source the setup.bash to launch the launch file.

## Map that's build can be saved using the below commands in map folder.
rosrun map_saver map_server -f name_for_the_map 

This command creates name_for_the_map.pgm and name_for_the_map.yaml in the map folder.
