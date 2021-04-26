catkin_make --source Prometheus_control/common/msgs --build build/prometheus_msgs
catkin_make --source uavros_msgs --build build/uavros_msgs
catkin_make --source Prometheus_control/control --build build/prometheus_control
catkin_make --source uavros_simulation/uavros_gazebo --build build/uavros_gazebo
catkin_make --source uavros_experiment/ARtagLanding --build build/uavros_artaglanding
catkin_make --source uavros_experiment/uavros_uavugv --build build/uavros_uavugv
catkin_make --source uavros_simulation/kcffollow_simulation --build build/uavros_kcffollow_simulation
catkin_make --source uavros_simulation/uavros_uavugv_sitl --build build/uavros_uavugv_sitl

#to be modified
#catkin_make --source Modules/object_detection --build build/object_detection
#catkin_make --source Modules/mission --build build/mission
#catkin_make --source Modules/slam --build build/slam
#catkin_make --source Modules/planning --build build/planning
#catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
#catkin_make --source Experiment --build build/prometheus_experiment


