export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD/plugin/build

gazebo --verbose plugin/environment.world
