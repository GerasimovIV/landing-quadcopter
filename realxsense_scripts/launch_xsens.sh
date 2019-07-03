roslaunch rs_camera.launch & 
sleep 5s &
roslaunch xsens_mti_node.launch &
sleep 5s &
./for_imu_added_vis_cov_spike.py &
roslaunch imu_added_rs_rtabmap.launch rtabmapviz:=false
