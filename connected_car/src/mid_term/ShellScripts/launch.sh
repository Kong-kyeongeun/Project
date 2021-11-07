#! /bin/bash
gnome-terminal -e "roscore" &
sleep 3
gnome-terminal -e "rosrun mid_term car_1" &
sleep 0.5
gnome-terminal -e "rosrun mid_term car_2" &
sleep 0.5
gnome-terminal -e "rosrun mid_term car_3" &
sleep 0.5
gnome-terminal -e "rosrun mid_term control_center" &
sleep 0.5
gnome-terminal -e "rosrun mid_term police_office" &
sleep 0.5
gnome-terminal -e "rosrun mid_term home.py" &
sleep 0.5
gnome-terminal -e "rosrun mid_term simulator" &
sleep 0.5
gnome-terminal -e "rqt_graph"
