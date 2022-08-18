# ftm_nov
let's run how the  drone fly
ez px4 launch : roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

PX4_HOME_LAT=37.547453 PX4_HOME_LON=127.078450 PX4_HOME_ALT=28.5 COM_RC_IN_MODE=1 NAV_RCL_ACT=0 make px4_sitl_default gazebo

ez lat&long modifiy
