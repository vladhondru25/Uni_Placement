# Uni_Placement

alias joyset='ls -l /dev/input/js0 && rosparam set joy_node/dev "/dev/input/js0" && rosrun joy joy_node _coalesce_interval:=0.05'
alias liset='ls -l /dev |grep ttyUSB && sudo chmod 666 /dev/ttyUSB0'
alias joyrun='rosrun mallard_py_joy teleop_joy.py'
