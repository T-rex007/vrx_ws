source /opt/ros/noetic/setup.bash
source ~/vrx_ws/devel/setup.bash
roslaunch vrx_gazebo sydneyregatta.launch urdf:=$HOME/vrx_ws/src/wamv/wamv.urdf gps_enabled:=true 
