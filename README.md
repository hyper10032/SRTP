# Autonomous elevator boarding robot by recognizing and pushing the button
This is a Student Research Training Program (SRTP) to build an autonomous elevator boarding robot. The robot, whcih combines a laser, a 6-axis innfos robot arm, an EAI Dashgo D1 robot base and an Xtion Pro RGB-D camera, is able to autonomously navigate in an office building and takes the elevator to enter different floors. A demo is available [here](https://www.bilibili.com/video/BV1qo4y117FL#reply4635072683).

## About the robot arm
1. The arm will get the button pose from the camera, plan a path to touch the button and execute it.
2. The official installation instruction is [here](http://wiki.mintasca.cloudminds.com/wiki/cn/index.html#!pages/ros_gluon.md). Follow the README.
3. Before starting the launch file, the arm is required to be held in zero-positon. And it is important to start the launch file twice before you can control it. We don't know why.
4. Launch the arm.
```bash
roslaunch gluon_moveit_config cm_demo.launch
rosrun mynavigation arm_pub.py
```

## About the camera
1. The camera will detect the Aruco tag and send the button pose.
2. The camera is required to launch after the arm. We don't know why.
3. Camera calibration is necessary. Follow the instruction [here](https://www.google.com). After calibration, change the parameters in 45_ws/src/mydetector/config. Our calibration result is in docs directory.
4. Launch the camera.
```bash
roslaunch mydetector goaldetector.launch
```

## About the robot base and the lidar
1. The specification PDFs are in docs directory.
2. Warning: we modify the urg_node package to eliminate some points, whcih is inappropriate and should be aware of.
3. Launch the robot base and the lidar.
```bash
roslaunch dashgo_nav teb_amcl_demo.launch
rosrun mynavigation global_planner.py
rosrun mynavigation local_planner.py
rosrun mynavigation enter_elevator.py
```
4. We give the robot an initial pose and send a goal in terminal.
```bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{pose: {pose:{position: {x: -0.0144352912903, y: -0.534833431244, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999998088544, w: 0.00195522581005}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]   }}'
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{pose: {position: {x: -1.006852893829, y: 1.80455803871, z: 1}, orientation: {x: 0, y: 0, z: 0, w: 0}}}'
```

## Backup
在 /base_link 前面的 6 个参数,只需要调整前面 5 个参数,最后一个默认为 0.0 即可,
前面 3 个,分别表示激光雷达 F4 在 X、Y、Z 轴(右手定则)上距离(0 , 0 , 0)点的坐标位置,(0 , 0 , 0)点是 D1 的坐标系原点,该点是 D1 的重心点。
后面 2 个,分别表示沿着 D1 中心线(正前方与正后方连成的直线)左右方向、上下方向移动偏离的角度,大小范围为 -3.1415926 ~ 3.1415926 , -3.1415926 为-180
度, 3.1415926 为 180 度。

注意相机坐标系与机械臂坐标系。
```bash
rosrun dashgo_bringup teleop_twist_keyboard.py
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
rostopic pub /enter geometry_msgs/Pose '{position: {x: 0.0, y: 0, z: 1}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
rostopic echo /enter geometry_msgs/Pose '{position: {x: 0.0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
```