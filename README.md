Termina-1: launch the gazebo
>>  ros2 launch so101_gazebo so101_gazebo.launch.py moveit:=follower
Terminal-2: launch the topic bridge for camera data service
>>   ros2 run ros_gz_bridge parameter_bridge "/d435i/image@sensor_msgs/msg/Image[gz.msgs.Image"
Terminal-3: launch the script file
>>   python3 src/so101_gazebo/scripts/object_detector.py --ros-args -p use_sim_time:=true
video link: https://drive.google.com/file/d/1yoxOQozK_AeCi0t0n08H1MoVFJop0kSF/view?usp=drive_link
