gnome-terminal --tab -e 'bash -c "roslaunch uavros_wrzf_sitl cam_track.launch; exec bash"' \
--tab -e 'bash -c "sleep 1;rosrun car_video car_tracking.py; exec bash"' \
