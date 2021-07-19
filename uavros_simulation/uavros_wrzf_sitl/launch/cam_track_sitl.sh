gnome-terminal --tab -e 'bash -c "roslaunch uavros_wrzf_sitl cam_track_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 1;rosrun car_video car_tracking_sitl.py; exec bash"' \
--tab -e 'bash -c "sleep 1;cd ~/UAVros/uavros_simulation/uavros_wrzf_sitl/script && python3 uav_ac.py; exec bash"' \
