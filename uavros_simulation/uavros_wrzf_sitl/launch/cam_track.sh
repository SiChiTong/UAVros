gnome-terminal --tab -e 'bash -c "roslaunch uavros_wrzf_sitl cam_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 1;rosrun uavros_wrzf_sitl car_tracking.py; exec bash"' \
--tab -e 'bash -c "sleep 1;cd ~/UAVros/uavros_simulation/uavros_wrzf_sitl/script && python3 uav_ac.py; exec bash"' \
