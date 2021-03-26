##ar-tag precision landing
gnome-terminal --tab -e 'bash -c "roscore; exec bash"' \
--window --title="Landing_control" -e 'bash -c "sleep 2; roslaunch uavros_artaglanding p200_precision_landing.launch; exec bash"' --geometry 10x24+10+10 \
--window --title="Mavros node" -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' --geometry 70x10+10+700 \
--window --title="/mavros/setpoint_raw" -e 'bash -c "sleep 2; rostopic echo /mavros/setpoint_raw/local; exec bash"' --geometry 10x24+400+10 \
--window --title="keyboard_control_px4.py" -e 'bash -c "sleep 2; rosrun uavros_artaglanding keyboard_control_px4.py; exec bash"' --geometry 10x24+800+10 \
--window --title="/visualization_marker" -e 'bash -c "sleep 2; rostopic echo /visualization_marker; exec bash"' --geometry 10x34+1200+10 \
--window -e 'bash -c "sleep 2; rqt_image_view; exec bash"' --geometry 5x5+10+800 \




