##ar-tag precision landing
gnome-terminal --window -e 'bash -c "roscore; exec bash"' --geometry 50x10+1300+700 \
--window -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' --geometry 70x10+70+600 \
--window -e 'bash -c "sleep 5; roslaunch uavros_artaglanding p200_precision_landing.launch; exec bash"' --geometry 25x24+10+10 \
--window -e 'bash -c "sleep 4; rosrun uavros_artaglanding keyboard_control_px4.py; exec bash"' --geometry 25x24+800+10 \
--window -e 'bash -c "sleep 4; rostopic echo /mavros/setpoint_raw/local; exec bash"' --geometry 25x24+400+10 \
--window -e 'bash -c "sleep 4; rostopic echo /visualization_marker; exec bash"' --geometry 25x34+1100+10 \
--window -e 'bash -c "sleep 4; rqt_image_view; exec bash"' --geometry 5x5+10+800 \
--window -e 'bash -c "sleep 4; rostopic echo /mavros/state; exec bash"' --geometry 25x15+1400+10 \
--window -e 'bash -c "sleep 4; rostopic echo /mavros/local_position/pose; exec bash"' --geometry 25x24+1700+10 \
--window -e 'bash -c "sleep 4; rostopic echo /mavros/global_position/rel_alt; exec bash"' --geometry 25x10+1700+600 \



