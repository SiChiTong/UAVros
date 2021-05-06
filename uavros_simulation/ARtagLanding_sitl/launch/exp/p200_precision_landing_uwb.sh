##ar-tag precision landing
gnome-terminal --window -e 'bash -c "roslaunch uavros_artaglanding launch_all.launch; exec bash"' --geometry 75x20+10+10 \
--window -e 'bash -c "sleep 5; rostopic echo /visualization_marker; exec bash"' --geometry 25x34+750+10 \
--window -e 'bash -c "sleep 5; rostopic echo /mavros/local_position/pose; exec bash"' --geometry 25x24+960+10 \
--window -e 'bash -c "sleep 5; rqt_image_view; exec bash"' --geometry 5x5+700+700 \



