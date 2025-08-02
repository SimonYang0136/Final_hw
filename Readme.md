cd到Gazebo_ws下
rviz(launch一键启动未配置)
rosrun homework mapper 启动建图节点
rosrun homework_planning planning 启动规划节点
rosrun control box_controller 启动控制节点
roslaunch gazebo_sim spawn_box_urdf2.launch 启动gazebo仿真(自建的地图在world文件夹里，由于启动报错还没加进去)
rosbag play -l src/fsd_common_msgs/bag/vline.bag 播放建图车数据包
