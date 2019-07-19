reset
#source /opt/ros/melodic/setup.bash
#export LD_LIBRARY_PATH="/opt/ros/melodic/lib:/home/stsc/work/ros_ws/sitl_ipc/lib"
gazebo --verbose -s libNazePlugin.so /usr/share/gazebo-9/worlds/naze.world 
