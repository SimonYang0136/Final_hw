// #include <ros/ros.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <cmath>
// #include <vector>
// #include <fsd_common_msgs/ConeDetections.h>
// #include <fsd_common_msgs/CarState.h>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// class ConeMapper {
// public:
//     ConeMapper() : tf_listener_(tf_buffer_) {
//         // 订阅锥桶检测和SLAM状态
//         sub_cones_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &ConeMapper::coneCallback, this);
//         sub_slam_ = nh_.subscribe("/estimation/slam/state", 10, &ConeMapper::slamCallback, this);
        
//         // 发布建图结果
//         pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/mapping/cones", 10);
        
//         // 初始化锥桶尺寸
//         red_cone_scale_.x = 0.5;   // 增大尺寸使其更易观察
//         red_cone_scale_.y = 0.5;
//         red_cone_scale_.z = 0.8;
//         blue_cone_scale_ = red_cone_scale_;
//         yellow_cone_scale_ = red_cone_scale_;
        
//         // 初始化变换矩阵为单位矩阵
//         vehicle_to_map_transform_.setIdentity();
        
//         ROS_INFO("ConeMapper initialized");
//     }

//     void slamCallback(const fsd_common_msgs::CarState::ConstPtr& msg) {
//         // 更新车辆位姿
//         current_pose_.position.x = msg->car_state.x;
//         current_pose_.position.y = msg->car_state.y;
        
//         // 创建四元数表示偏航角
//         tf2::Quaternion q;
//         q.setRPY(0, 0, msg->car_state.theta);
//         current_pose_.orientation = tf2::toMsg(q);
        
//         // 更新变换矩阵
//         Eigen::Translation3d translation(msg->car_state.x, msg->car_state.y, 0);
//         Eigen::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
//         vehicle_to_map_transform_ = translation * rotation;
        
//         // 发布车辆位姿
//         publishVehiclePose();
        
//         ROS_DEBUG_THROTTLE(1.0, "SLAM update: x=%.2f, y=%.2f, theta=%.2f", 
//                           msg->car_state.x, msg->car_state.y, msg->car_state.theta);
//     }

//     void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
//         visualization_msgs::MarkerArray marker_array;
        
//         // 添加删除所有旧标记的指令
//         visualization_msgs::Marker delete_all;
//         delete_all.action = visualization_msgs::Marker::DELETEALL;
//         delete_all.ns = "cones";
//         marker_array.markers.push_back(delete_all);
        
//         // 遍历所有检测到的锥桶
//         for (size_t i = 0; i < msg->cone_detections.size(); ++i) {
//             const auto& cone = msg->cone_detections[i];
            
//             // 雷达坐标系下的点（考虑雷达在车辆前方的偏移）
//             // 雷达安装在车辆前方2.4米处，所以需要加上偏移量
//             Eigen::Vector3d cone_in_vehicle(
//                 cone.position.x + 2.4, 
//                 cone.position.y, 
//                 0.0  // 忽略Z轴高度
//             );
            
//             // 转换到地图坐标系
//             Eigen::Vector3d cone_in_map = vehicle_to_map_transform_ * cone_in_vehicle;
            
//             // 创建Marker对象
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = ros::Time::now();
//             marker.ns = "cones";
//             marker.id = i;
//             marker.type = visualization_msgs::Marker::CYLINDER;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.pose.position.x = cone_in_map.x();
//             marker.pose.position.y = cone_in_map.y();
//             marker.pose.position.z = 0.0;
//             marker.pose.orientation.w = 1.0;
            
//             // 根据颜色设置属性
//             if (cone.color.data == "r") {
//                 marker.color.r = 1.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = red_cone_scale_;
//             } else if (cone.color.data == "b") {
//                 marker.color.r = 0.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 1.0;
//                 marker.color.a = 0.8;
//                 marker.scale = blue_cone_scale_;
//             } else {
//                 // 默认黄色
//                 marker.color.r = 1.0;
//                 marker.color.g = 1.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = yellow_cone_scale_;
//             }
            
//             marker.lifetime = ros::Duration(30.0);
//             marker_array.markers.push_back(marker);
            
//             ROS_DEBUG("Cone %zu: vehicle(%.2f,%.2f) -> map(%.2f,%.2f)", 
//                      i, cone_in_vehicle.x(), cone_in_vehicle.y(), 
//                      cone_in_map.x(), cone_in_map.y());
//         }
        
//         // 发布标记信息
//         pub_markers_.publish(marker_array);
//         ROS_INFO_THROTTLE(1.0, "Published %zu cones", msg->cone_detections.size());
//     }

// private:
//     void publishVehiclePose() {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped.header.stamp = ros::Time::now();
//         transformStamped.header.frame_id = "map";
//         transformStamped.child_frame_id = "base_link";
        
//         transformStamped.transform.translation.x = current_pose_.position.x;
//         transformStamped.transform.translation.y = current_pose_.position.y;
//         transformStamped.transform.translation.z = 0.0;
//         transformStamped.transform.rotation = current_pose_.orientation;
        
//         tf_broadcaster_.sendTransform(transformStamped);
//     }

//     // 需要用到的变量
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_cones_;
//     ros::Subscriber sub_slam_;
//     ros::Publisher pub_markers_;
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
//     tf2_ros::TransformBroadcaster tf_broadcaster_;
    
//     geometry_msgs::Pose current_pose_;
//     geometry_msgs::Vector3 red_cone_scale_, blue_cone_scale_, yellow_cone_scale_;
    
//     // 车辆到地图的变换矩阵
//     Eigen::Affine3d vehicle_to_map_transform_;
// };

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "cone_mapper");
//     ConeMapper mapper;
//     ros::spin();
//     return 0;
// }

// #include <ros/ros.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <cmath>
// #include <vector>
// #include <fsd_common_msgs/ConeDetections.h>
// #include <fsd_common_msgs/CarState.h>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <std_msgs/Float32.h>

// class ConeMapper {
// public:
//     ConeMapper() : tf_listener_(tf_buffer_) {
//         // 初始化节点句柄
//         nh_ = ros::NodeHandle("~");
        
//         // 订阅锥桶检测和SLAM状态
//         sub_cones_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &ConeMapper::coneCallback, this);
//         sub_slam_ = nh_.subscribe("/estimation/slam/state", 10, &ConeMapper::slamCallback, this);
        
//         // 发布建图结果
//         pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/mapping/cones", 10);
        
//         // 调试发布
//         pub_test_ = nh_.advertise<visualization_msgs::Marker>("/debug/test_marker", 10);
//         pub_debug_ = nh_.advertise<std_msgs::Float32>("/debug/slam_data", 10);
        
//         // 初始化锥桶尺寸
//         red_cone_scale_.x = 0.5;   // 增大尺寸使其更易观察
//         red_cone_scale_.y = 0.5;
//         red_cone_scale_.z = 0.8;
//         blue_cone_scale_ = red_cone_scale_;
//         yellow_cone_scale_ = red_cone_scale_;
        
//         // 初始化变换矩阵为单位矩阵
//         vehicle_to_map_transform_.setIdentity();
        
//         // 初始化车辆位置
//         current_pose_.position.x = 0.0;
//         current_pose_.position.y = 0.0;
//         current_pose_.position.z = 0.0;
//         current_pose_.orientation.w = 1.0;
        
//         // 发布静态测试标记
//         publishTestMarker();
        
//         ROS_INFO("ConeMapper initialized with debug features");
//     }

//     void slamCallback(const fsd_common_msgs::CarState::ConstPtr& msg) {
//         // 调试输出
//         std_msgs::Float32 debug_msg;
//         debug_msg.data = msg->car_state.x;
//         pub_debug_.publish(debug_msg);
        
//         ROS_INFO_THROTTLE(1.0, "SLAM update: x=%.2f, y=%.2f, theta=%.2f", 
//                           msg->car_state.x, msg->car_state.y, msg->car_state.theta);
        
//         // 更新车辆位姿
//         current_pose_.position.x = msg->car_state.x;
//         current_pose_.position.y = msg->car_state.y;
        
//         // 创建四元数表示偏航角
//         tf2::Quaternion q;
//         q.setRPY(0, 0, msg->car_state.theta);
//         current_pose_.orientation = tf2::toMsg(q);
        
//         // 更新变换矩阵
//         Eigen::Translation3d translation(msg->car_state.x, msg->car_state.y, 0);
//         Eigen::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
//         vehicle_to_map_transform_ = translation * rotation;
        
//         // 发布车辆位姿
//         publishVehiclePose();
        
//         // 发布位置测试标记
//         publishPositionTestMarker();
//     }

//     void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
//         ROS_INFO_THROTTLE(1.0, "Received %zu cone detections", msg->cone_detections.size());
        
//         visualization_msgs::MarkerArray marker_array;
        
//         // 添加删除所有旧标记的指令
//         visualization_msgs::Marker delete_all;
//         delete_all.action = visualization_msgs::Marker::DELETEALL;
//         delete_all.ns = "cones";
//         marker_array.markers.push_back(delete_all);
        
//         // 如果没有任何锥桶检测，发布空数组并返回
//         if (msg->cone_detections.empty()) {
//             pub_markers_.publish(marker_array);
//             ROS_WARN_THROTTLE(1.0, "No cones detected!");
//             return;
//         }
        
//         // 遍历所有检测到的锥桶
//         for (size_t i = 0; i < msg->cone_detections.size(); ++i) {
//             const auto& cone = msg->cone_detections[i];
            
//             ROS_DEBUG("Cone %zu: raw position (%.2f, %.2f)", i, cone.position.x, cone.position.y);
            
//             // 雷达坐标系下的点（考虑雷达在车辆前方的偏移）
//             // 雷达安装在车辆前方2.4米处，所以需要加上偏移量
//             Eigen::Vector3d cone_in_vehicle(
//                 cone.position.x + 2.4, 
//                 cone.position.y, 
//                 0.0  // 忽略Z轴高度
//             );
//             Eigen::Vector3d distance(11445.68,21664.22,0.0);
//             // 转换到地图坐标系
//             Eigen::Vector3d cone_in_map = vehicle_to_map_transform_ * cone_in_vehicle+distance;
            
//             ROS_DEBUG("Cone %zu: vehicle(%.2f,%.2f) -> map(%.2f,%.2f)", 
//                      i, cone_in_vehicle.x(), cone_in_vehicle.y(), 
//                      cone_in_map.x(), cone_in_map.y());
            
//             // 创建Marker对象
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = ros::Time::now();
//             marker.ns = "cones";
//             marker.id = i;
//             marker.type = visualization_msgs::Marker::CYLINDER;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.pose.position.x = cone_in_map.x();
//             marker.pose.position.y = cone_in_map.y();
//             marker.pose.position.z = 0.0;
//             marker.pose.orientation.w = 1.0;
            
//             // 根据颜色设置属性
//             if (cone.color.data == "r") {
//                 marker.color.r = 1.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = red_cone_scale_;
//             } else if (cone.color.data == "b") {
//                 marker.color.r = 0.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 1.0;
//                 marker.color.a = 0.8;
//                 marker.scale = blue_cone_scale_;
//             } else {
//                 // 默认黄色
//                 marker.color.r = 1.0;
//                 marker.color.g = 1.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = yellow_cone_scale_;
//             }
            
//             marker.lifetime = ros::Duration(0); // 永久显示
//             marker_array.markers.push_back(marker);
//         }
        
//         // 添加原点测试标记
//         visualization_msgs::Marker origin_marker;
//         origin_marker.header.frame_id = "map";
//         origin_marker.header.stamp = ros::Time::now();
//         origin_marker.ns = "origin";
//         origin_marker.id = 9998;
//         origin_marker.type = visualization_msgs::Marker::SPHERE;
//         origin_marker.action = visualization_msgs::Marker::ADD;
//         origin_marker.pose.position.x = 0;
//         origin_marker.pose.position.y = 0;
//         origin_marker.pose.position.z = 0;
//         origin_marker.scale.x = 1.0;
//         origin_marker.scale.y = 1.0;
//         origin_marker.scale.z = 1.0;
//         origin_marker.color.r = 0.0;
//         origin_marker.color.g = 1.0;
//         origin_marker.color.b = 0.0;
//         origin_marker.color.a = 1.0;
//         origin_marker.lifetime = ros::Duration(0);
//         marker_array.markers.push_back(origin_marker);
        
//         // 发布标记信息
//         pub_markers_.publish(marker_array);
//         ROS_INFO("Published %zu cones in map frame", msg->cone_detections.size());
//     }

// private:
//     void publishVehiclePose() {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped.header.stamp = ros::Time::now();
//         transformStamped.header.frame_id = "map";
//         transformStamped.child_frame_id = "base_link";
        
//         transformStamped.transform.translation.x = current_pose_.position.x;
//         transformStamped.transform.translation.y = current_pose_.position.y;
//         transformStamped.transform.translation.z = 0.0;
//         transformStamped.transform.rotation = current_pose_.orientation;
        
//         tf_broadcaster_.sendTransform(transformStamped);
//         ROS_DEBUG("Published vehicle pose: x=%.2f, y=%.2f", 
//                  current_pose_.position.x, current_pose_.position.y);
//     }
    
//     void publishTestMarker() {
//         // 发布一个固定在map坐标系原点的测试标记
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "test";
//         marker.id = 9999;
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.position.x = 0;
//         marker.pose.position.y = 0;
//         marker.pose.position.z = 0;
//         marker.scale.x = 1.0;
//         marker.scale.y = 1.0;
//         marker.scale.z = 1.0;
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;
//         marker.color.a = 1.0;
//         marker.lifetime = ros::Duration(0); // 永久显示
        
//         pub_test_.publish(marker);
//         ROS_INFO("Published test marker at map origin");
//     }
    
//     void publishPositionTestMarker() {
//         // 发布一个在车辆位置的测试标记
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "vehicle_position";
//         marker.id = 10000;
//         marker.type = visualization_msgs::Marker::CUBE;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.position.x = current_pose_.position.x;
//         marker.pose.position.y = current_pose_.position.y;
//         marker.pose.position.z = 0;
//         marker.scale.x = 0.5;
//         marker.scale.y = 0.5;
//         marker.scale.z = 0.5;
//         marker.color.r = 0.0;
//         marker.color.g = 0.0;
//         marker.color.b = 1.0;
//         marker.color.a = 1.0;
//         marker.lifetime = ros::Duration(1.0); // 临时显示
        
//         pub_test_.publish(marker);
//         ROS_DEBUG("Published vehicle position marker at (%.2f, %.2f)", 
//                  current_pose_.position.x, current_pose_.position.y);
//     }

//     // 需要用到的变量
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_cones_;
//     ros::Subscriber sub_slam_;
//     ros::Publisher pub_markers_;
//     ros::Publisher pub_test_;
//     ros::Publisher pub_debug_;
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
//     tf2_ros::TransformBroadcaster tf_broadcaster_;
    
//     geometry_msgs::Pose current_pose_;
//     geometry_msgs::Vector3 red_cone_scale_, blue_cone_scale_, yellow_cone_scale_;
    
//     // 车辆到地图的变换矩阵
//     Eigen::Affine3d vehicle_to_map_transform_;
// };

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "cone_mapper");
    
//     // 设置日志级别为DEBUG
//     if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//         ros::console::notifyLoggerLevelsChanged();
//     }
    
//     ConeMapper mapper;
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <fsd_common_msgs/ConeDetections.h>
#include <fsd_common_msgs/CarState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float32.h>

class ConeMapper {
public:
    ConeMapper() : tf_listener_(tf_buffer_) {
        // 初始化节点句柄
        nh_ = ros::NodeHandle("~");
        
        // 读取参数
        nh_.param("cone_lifetime", cone_lifetime_, 30.0);
        nh_.param("max_distance", max_distance_, 100.0);
        nh_.param("lidar_offset_x", lidar_offset_x_, 2.4);
        
        ROS_INFO("Parameters: cone_lifetime=%.1f, max_distance=%.1f, lidar_offset_x=%.1f", 
                cone_lifetime_, max_distance_, lidar_offset_x_);
        
        // 订阅锥桶检测和SLAM状态
        sub_cones_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &ConeMapper::coneCallback, this);
        sub_slam_ = nh_.subscribe("/estimation/slam/state", 10, &ConeMapper::slamCallback, this);
        
        // 发布建图结果
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/mapping/cones", 10);
        pub_global_cones_ = nh_.advertise<geometry_msgs::PoseArray>("/global_cones", 10);
        
        // 调试发布
        pub_debug_cones_ = nh_.advertise<geometry_msgs::PoseArray>("/debug/raw_cones", 10);
        pub_debug_info_ = nh_.advertise<std_msgs::Float32>("/debug/cone_count", 10);
        
        // 初始化锥桶尺寸
        red_cone_scale_.x = 0.2;   // 增大尺寸使其更易观察
        red_cone_scale_.y = 0.2;
        red_cone_scale_.z = 0.3;
        blue_cone_scale_ = red_cone_scale_;
        yellow_cone_scale_ = red_cone_scale_;
        
        // 初始化变换矩阵为单位矩阵
        vehicle_to_map_transform_.setIdentity();
        
        // 初始化更新时间
        last_publish_time_ = ros::Time::now();
        last_slam_update_time_ = ros::Time(0);  // 初始化为0时间
        last_cone_update_time_ = ros::Time(0);
        
        ROS_INFO("ConeMapper initialized with enhanced debugging");
    }

    void slamCallback(const fsd_common_msgs::CarState::ConstPtr& msg) {
        // 更新SLAM时间戳
        last_slam_update_time_ = ros::Time::now();
        
        // 更新车辆位姿
        current_pose_.position.x = msg->car_state.x;
        current_pose_.position.y = msg->car_state.y;
        
        // 创建四元数表示偏航角
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->car_state.theta);
        current_pose_.orientation = tf2::toMsg(q);
        
        // 更新变换矩阵
        Eigen::Translation3d translation(msg->car_state.x, msg->car_state.y, 0);
        Eigen::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
        vehicle_to_map_transform_ = translation * rotation;
        
        // 发布车辆位姿
        publishVehiclePose();
        
        ROS_DEBUG_THROTTLE(1.0, "SLAM update: x=%.2f, y=%.2f, theta=%.2f", 
                          msg->car_state.x, msg->car_state.y, msg->car_state.theta);
        
        // 定期发布锥桶
        if ((ros::Time::now() - last_publish_time_).toSec() > 0.1) {
            publishCones();
            last_publish_time_ = ros::Time::now();
        }
    }

    void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
        ros::Time now = ros::Time::now();
        last_cone_update_time_ = now;
        
        // 发布原始锥桶数据用于调试
        geometry_msgs::PoseArray raw_cones;
        raw_cones.header = msg->header;
        for (const auto& cone : msg->cone_detections) {
            geometry_msgs::Pose pose;
            pose.position = cone.position;
            raw_cones.poses.push_back(pose);
        }
        pub_debug_cones_.publish(raw_cones);
        
        ROS_INFO_THROTTLE(1.0, "Received %zu raw cone detections", msg->cone_detections.size());
        
        // 如果没有车辆位姿信息，跳过处理
        if (last_slam_update_time_.isZero() || (now - last_slam_update_time_).toSec() > 1.0) {
            ROS_WARN_THROTTLE(1.0, "No recent SLAM data (%.2f sec old)", 
                             last_slam_update_time_.isZero() ? -1.0 : (now - last_slam_update_time_).toSec());
            return;
        }
        
        // 遍历所有检测到的锥桶
        for (size_t i = 0; i < msg->cone_detections.size(); ++i) {
            const auto& cone = msg->cone_detections[i];
            
            // 雷达坐标系下的点（考虑雷达在车辆前方的偏移）
            Eigen::Vector3d cone_in_vehicle(
                cone.position.x + lidar_offset_x_, 
                cone.position.y, 
                0.0  // 忽略Z轴高度
            );
            Eigen::Vector3d Distance(11445.68,21664.22,0.0);
            // 转换到地图坐标系
            Eigen::Vector3d cone_in_map = vehicle_to_map_transform_ * cone_in_vehicle+Distance;
            
            ROS_DEBUG("Cone %zu: raw(%.2f,%.2f) -> vehicle(%.2f,%.2f) -> map(%.2f,%.2f)", 
                     i, cone.position.x, cone.position.y,
                     cone_in_vehicle.x(), cone_in_vehicle.y(),
                     cone_in_map.x(), cone_in_map.y());
            
            // 过滤远距离锥桶
            double distance = sqrt(pow(cone_in_map.x(), 2) + pow(cone_in_map.y(), 2));
            if (distance > max_distance_) {
                ROS_DEBUG("Cone %zu filtered by distance: %.2f > %.2f", i, distance, max_distance_);
                continue;
            }
            
            // 创建锥桶的唯一ID (基于位置)
            size_t id = static_cast<size_t>(cone_in_map.x() * 100) * 100000 + 
                        static_cast<size_t>(cone_in_map.y() * 100);
            
            // 检查是否已存在该锥桶
            if (global_cones_.find(id) == global_cones_.end()) {
                // 新锥桶
                ConeData new_cone;
                new_cone.position.x = cone_in_map.x();
                new_cone.position.y = cone_in_map.y();
                new_cone.position.z = cone_in_map.z();
                new_cone.color = cone.color.data;
                new_cone.last_seen = now;
                global_cones_[id] = new_cone;
                
                ROS_DEBUG("Added new cone ID %zu at (%.2f, %.2f)", id, cone_in_map.x(), cone_in_map.y());
            } else {
                // 更新现有锥桶（使用加权平均）
                const double alpha = 0.3; // 平滑因子
                global_cones_[id].position.x = 
                    alpha * cone_in_map.x() + (1 - alpha) * global_cones_[id].position.x;
                global_cones_[id].position.y = 
                    alpha * cone_in_map.y() + (1 - alpha) * global_cones_[id].position.y;
                global_cones_[id].last_seen = now;
                
                ROS_DEBUG("Updated cone ID %zu at (%.2f, %.2f)", id, global_cones_[id].position.x, global_cones_[id].position.y);
            }
        }
        
        // 发布锥桶计数
        std_msgs::Float32 count_msg;
        count_msg.data = global_cones_.size();
        pub_debug_info_.publish(count_msg);
    }

    void publishCones() {
        visualization_msgs::MarkerArray marker_array;
        geometry_msgs::PoseArray global_poses;
        global_poses.header.stamp = ros::Time::now();
        global_poses.header.frame_id = "map";
        
        // 添加删除所有旧标记的指令
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        delete_all.ns = "cones";
        marker_array.markers.push_back(delete_all);
        
        // 遍历全局锥桶
        int id = 0;
        ros::Time now = ros::Time::now();
        for (auto it = global_cones_.begin(); it != global_cones_.end();) {
            // 检查锥桶是否过期
            if ((now - it->second.last_seen).toSec() > cone_lifetime_) {
                ROS_DEBUG("Removing expired cone ID %zu", it->first);
                it = global_cones_.erase(it);
                continue;
            }
            
            // 创建Marker对象
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = "cones";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = it->second.position;
            marker.pose.orientation.w = 1.0;
            
            // 根据颜色设置属性
            if (it->second.color == "r") {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
                marker.scale = red_cone_scale_;
            } else if (it->second.color == "b") {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 0.8;
                marker.scale = blue_cone_scale_;
            } else {
                // 默认黄色
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
                marker.scale = yellow_cone_scale_;
            }
            
            marker.lifetime = ros::Duration(cone_lifetime_);
            marker_array.markers.push_back(marker);
            
            // 添加到全局锥桶列表
            geometry_msgs::Pose pose;
            pose.position = it->second.position;
            global_poses.poses.push_back(pose);
            
            ++it;
        }
        
        // 发布标记信息
        if (!marker_array.markers.empty()) {
            pub_markers_.publish(marker_array);
            ROS_INFO_THROTTLE(1.0, "Published %zu cones in map frame", global_cones_.size());
        } else {
            ROS_WARN_THROTTLE(1.0, "No cones to publish");
        }
        
        if (!global_poses.poses.empty()) {
            pub_global_cones_.publish(global_poses);
        }
        
        // 发布状态信息
        ROS_INFO_THROTTLE(5.0, "Mapper Status: Cones=%zu | Last SLAM=%.1fs ago | Last Cones=%.1fs ago",
                         global_cones_.size(),
                         last_slam_update_time_.isZero() ? -1.0 : (now - last_slam_update_time_).toSec(),
                         last_cone_update_time_.isZero() ? -1.0 : (now - last_cone_update_time_).toSec());
    }

private:
    struct ConeData {
        geometry_msgs::Point position;
        std::string color;
        ros::Time last_seen;
    };

    void publishVehiclePose() {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        
        transformStamped.transform.translation.x = current_pose_.position.x;
        transformStamped.transform.translation.y = current_pose_.position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = current_pose_.orientation;
        
        try {
            tf_broadcaster_.sendTransform(transformStamped);
            ROS_DEBUG("Published vehicle TF: x=%.2f, y=%.2f", 
                     current_pose_.position.x, current_pose_.position.y);
        } catch (const std::exception& e) {
            ROS_ERROR("TF broadcast error: %s", e.what());
        }
    }

    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber sub_cones_;
    ros::Subscriber sub_slam_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_global_cones_;
    ros::Publisher pub_debug_cones_;
    ros::Publisher pub_debug_info_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 锥桶数据
    std::unordered_map<size_t, ConeData> global_cones_;
    geometry_msgs::Pose current_pose_;
    ros::Time last_publish_time_;
    ros::Time last_slam_update_time_;
    ros::Time last_cone_update_time_;
    
    // 参数
    double cone_lifetime_;
    double max_distance_;
    double lidar_offset_x_;
    
    // 锥桶尺寸
    geometry_msgs::Vector3 red_cone_scale_;
    geometry_msgs::Vector3 blue_cone_scale_;
    geometry_msgs::Vector3 yellow_cone_scale_;
    
    // 变换矩阵
    Eigen::Affine3d vehicle_to_map_transform_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cone_mapper");
    
    // 设置更高的日志级别以便调试
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ConeMapper mapper;
    ros::spin();
    return 0;
}

// #include <ros/ros.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <cmath>
// #include <vector>
// #include <unordered_map>
// #include <fsd_common_msgs/ConeDetections.h>
// #include <fsd_common_msgs/CarState.h>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <std_msgs/Float32.h>
// #include <algorithm>
// #include <queue>

// class ConeMapper {
// public:
//     ConeMapper() : tf_listener_(tf_buffer_) {
//         // 初始化节点句柄
//         nh_ = ros::NodeHandle("~");
        
//         // 读取参数
//         nh_.param("cone_lifetime", cone_lifetime_, 30.0);
//         nh_.param("max_distance", max_distance_, 100.0);
//         nh_.param("lidar_offset_x", lidar_offset_x_, 2.4);
//         nh_.param("position_merge_threshold", position_merge_threshold_, 0.5); // 位置合并阈值(米)
//         nh_.param("position_smoothing_factor", position_smoothing_factor_, 0.2); // 位置平滑因子
        
//         ROS_INFO("Parameters: cone_lifetime=%.1f, max_distance=%.1f, lidar_offset_x=%.1f, merge_thresh=%.2f, smooth_factor=%.2f",
//                 cone_lifetime_, max_distance_, lidar_offset_x_, position_merge_threshold_, position_smoothing_factor_);
        
//         // 订阅锥桶检测和SLAM状态
//         sub_cones_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &ConeMapper::coneCallback, this);
//         sub_slam_ = nh_.subscribe("/estimation/slam/state", 10, &ConeMapper::slamCallback, this);
        
//         // 发布建图结果
//         pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/mapping/cones", 10);
//         pub_global_cones_ = nh_.advertise<geometry_msgs::PoseArray>("/global_cones", 10);
        
//         // 初始化锥桶尺寸
//         red_cone_scale_.x = 0.5;   // 增大尺寸使其更易观察
//         red_cone_scale_.y = 0.5;
//         red_cone_scale_.z = 0.8;
//         blue_cone_scale_ = red_cone_scale_;
//         yellow_cone_scale_ = red_cone_scale_;
        
//         // 初始化变换矩阵为单位矩阵
//         vehicle_to_map_transform_.setIdentity();
        
//         // 初始化更新时间
//         last_publish_time_ = ros::Time::now();
//         last_slam_update_time_ = ros::Time(0);  // 初始化为0时间
//         last_cone_update_time_ = ros::Time(0);
        
//         ROS_INFO("ConeMapper initialized with ghosting solutions");
//     }

//     void slamCallback(const fsd_common_msgs::CarState::ConstPtr& msg) {
//         // 更新SLAM时间戳
//         last_slam_update_time_ = ros::Time::now();
        
//         // 更新车辆位姿
//         current_pose_.position.x = msg->car_state.x;
//         current_pose_.position.y = msg->car_state.y;
        
//         // 创建四元数表示偏航角
//         tf2::Quaternion q;
//         q.setRPY(0, 0, msg->car_state.theta);
//         current_pose_.orientation = tf2::toMsg(q);
        
//         // 更新变换矩阵
//         Eigen::Translation3d translation(msg->car_state.x, msg->car_state.y, 0);
//         Eigen::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
//         //Eigen::Vector3d Distance(11445.68,21664.22,0.0);
//         vehicle_to_map_transform_ = translation * rotation;//+Distance;
        
//         // 发布车辆位姿
//         publishVehiclePose();
        
//         // 定期发布锥桶
//         if ((ros::Time::now() - last_publish_time_).toSec() > 0.1) {
//             publishCones();
//             last_publish_time_ = ros::Time::now();
//         }
//     }

//     void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
//         ros::Time now = ros::Time::now();
//         last_cone_update_time_ = now;
        
//         // 如果没有车辆位姿信息，跳过处理
//         if (last_slam_update_time_.isZero() || (now - last_slam_update_time_).toSec() > 1.0) {
//             ROS_WARN_THROTTLE(1.0, "No recent SLAM data (%.1fs old)", 
//                              last_slam_update_time_.isZero() ? -1.0 : (now - last_slam_update_time_).toSec());
//             return;
//         }
        
//         // 遍历所有检测到的锥桶
//         for (size_t i = 0; i < msg->cone_detections.size(); ++i) {
//             const auto& cone = msg->cone_detections[i];
            
//             // 雷达坐标系下的点（考虑雷达在车辆前方的偏移）
//             Eigen::Vector3d cone_in_vehicle(
//                 cone.position.x + lidar_offset_x_, 
//                 cone.position.y, 
//                 0.0  // 忽略Z轴高度
//             );
//             Eigen::Vector3d Distance(11445.68,21664.22,0.0);
//             // 转换到地图坐标系
//             Eigen::Vector3d cone_in_map = vehicle_to_map_transform_ * cone_in_vehicle+Distance;
            
//             // 过滤远距离锥桶
//             double distance = sqrt(pow(cone_in_map.x(), 2) + pow(cone_in_map.y(), 2));
//             if (distance > max_distance_) {
//                 continue;
//             }
            
//             // 尝试合并到现有锥桶
//             bool merged = false;
//             size_t nearest_id = 0;
//             double min_distance = std::numeric_limits<double>::max();
            
//             // 查找最近的锥桶
//             for (const auto& pair : global_cones_) {
//                 double dx = pair.second.position.x - cone_in_map.x();
//                 double dy = pair.second.position.y - cone_in_map.y();
//                 double dist = sqrt(dx*dx + dy*dy);
                
//                 if (dist < min_distance) {
//                     min_distance = dist;
//                     nearest_id = pair.first;
//                 }
//             }
            
//             // 如果距离小于阈值，合并锥桶
//             if (min_distance < position_merge_threshold_) {
//                 ConeData& existing_cone = global_cones_[nearest_id];
                
//                 // 使用指数平滑更新位置
//                 existing_cone.position.x = 
//                     position_smoothing_factor_ * cone_in_map.x() + 
//                     (1 - position_smoothing_factor_) * existing_cone.position.x;
                    
//                 existing_cone.position.y = 
//                     position_smoothing_factor_ * cone_in_map.y() + 
//                     (1 - position_smoothing_factor_) * existing_cone.position.y;
                
//                 existing_cone.last_seen = now;
//                 merged = true;
//                 ROS_DEBUG("Merged cone at (%.2f, %.2f) with existing cone ID %zu", 
//                          cone_in_map.x(), cone_in_map.y(), nearest_id);
//             }
            
//             // 如果没有合并，添加新锥桶
//             if (!merged) {
//                 // 创建锥桶的唯一ID (基于位置)
//                 size_t id = generateConeId(cone_in_map.x(), cone_in_map.y());
                
//                 ConeData new_cone;
//                 new_cone.position.x = cone_in_map.x();
//                 new_cone.position.y = cone_in_map.y();
//                 new_cone.position.z = cone_in_map.z();
//                 new_cone.color = cone.color.data;
//                 new_cone.last_seen = now;
//                 new_cone.detection_count = 1;
//                 global_cones_[id] = new_cone;
                
//                 ROS_DEBUG("Added new cone ID %zu at (%.2f, %.2f)", id, cone_in_map.x(), cone_in_map.y());
//             }
//         }
        
//         // 定期聚类锥桶（每10个锥桶检测执行一次）
//         static int detection_counter = 0;
//         if (++detection_counter >= 10) {
//             clusterCones();
//             detection_counter = 0;
//         }
//     }

//     void publishCones() {
//         visualization_msgs::MarkerArray marker_array;
//         geometry_msgs::PoseArray global_poses;
//         global_poses.header.stamp = ros::Time::now();
//         global_poses.header.frame_id = "map";
        
//         // 添加删除所有旧标记的指令
//         visualization_msgs::Marker delete_all;
//         delete_all.action = visualization_msgs::Marker::DELETEALL;
//         delete_all.ns = "cones";
//         marker_array.markers.push_back(delete_all);
        
//         // 遍历全局锥桶
//         int marker_id = 0;
//         ros::Time now = ros::Time::now();
//         for (auto it = global_cones_.begin(); it != global_cones_.end();) {
//             ConeData& cone = it->second;
            
//             // 检查锥桶是否过期
//             if ((now - cone.last_seen).toSec() > cone_lifetime_) {
//                 it = global_cones_.erase(it);
//                 continue;
//             }
            
//             // 创建Marker对象
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = now;
//             marker.ns = "cones";
//             marker.id = marker_id++;
//             marker.type = visualization_msgs::Marker::CYLINDER;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.pose.position = cone.position;
//             marker.pose.orientation.w = 1.0;
            
//             // 根据颜色设置属性
//             if (cone.color == "r") {
//                 marker.color.r = 1.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = red_cone_scale_;
//             } else if (cone.color == "b") {
//                 marker.color.r = 0.0;
//                 marker.color.g = 0.0;
//                 marker.color.b = 1.0;
//                 marker.color.a = 0.8;
//                 marker.scale = blue_cone_scale_;
//             } else {
//                 // 默认黄色
//                 marker.color.r = 1.0;
//                 marker.color.g = 1.0;
//                 marker.color.b = 0.0;
//                 marker.color.a = 0.8;
//                 marker.scale = yellow_cone_scale_;
//             }
            
//             marker.lifetime = ros::Duration(cone_lifetime_);
//             marker_array.markers.push_back(marker);
            
//             // 添加到全局锥桶列表
//             geometry_msgs::Pose pose;
//             pose.position = cone.position;
//             global_poses.poses.push_back(pose);
            
//             ++it;
//         }
        
//         // 发布标记信息
//         if (!marker_array.markers.empty()) {
//             pub_markers_.publish(marker_array);
//             ROS_INFO_THROTTLE(1.0, "Published %zu cones in map frame", global_cones_.size());
//         }
        
//         if (!global_poses.poses.empty()) {
//             pub_global_cones_.publish(global_poses);
//         }
//     }

// private:
//     struct ConeData {
//         geometry_msgs::Point position;
//         std::string color;
//         ros::Time last_seen;
//         int detection_count = 0; // 检测次数计数
//     };

//     //生成锥桶ID
//     size_t generateConeId(double x, double y) {
//         // 使用位置网格化生成稳定ID
//         int grid_x = static_cast<int>(x * 10); // 10厘米网格
//         int grid_y = static_cast<int>(y * 10);
        
//         // 使用位运算创建唯一ID
//         return (static_cast<size_t>(grid_x) << 32) | (static_cast<size_t>(grid_y) & 0xFFFFFFFF);
//     }

//     // 锥桶聚类算法
//     void clusterCones() {
//         ROS_DEBUG("Running cone clustering...");
        
//         // 简单的基于距离的聚类
//         std::vector<size_t> ids_to_remove;
//         std::vector<size_t> ids_to_keep;
        
//         for (auto& pair : global_cones_) {
//             ids_to_keep.push_back(pair.first);
//         }
        
//         // 按检测次数排序（检测次数多的更可能是真实锥桶）
//         std::sort(ids_to_keep.begin(), ids_to_keep.end(), 
//             [&](size_t a, size_t b) {
//                 return global_cones_[a].detection_count > global_cones_[b].detection_count;
//             });
        
//         // 遍历所有锥桶，合并邻近的
//         for (size_t i = 0; i < ids_to_keep.size(); i++) {
//             if (std::find(ids_to_remove.begin(), ids_to_remove.end(), ids_to_keep[i]) != ids_to_remove.end()) {
//                 continue; // 已经标记删除
//             }
            
//             ConeData& cone_i = global_cones_[ids_to_keep[i]];
            
//             for (size_t j = i + 1; j < ids_to_keep.size(); j++) {
//                 if (std::find(ids_to_remove.begin(), ids_to_remove.end(), ids_to_keep[j]) != ids_to_remove.end()) {
//                     continue; // 已经标记删除
//                 }
                
//                 ConeData& cone_j = global_cones_[ids_to_keep[j]];
                
//                 double dx = cone_i.position.x - cone_j.position.x;
//                 double dy = cone_i.position.y - cone_j.position.y;
//                 double distance = sqrt(dx*dx + dy*dy);
                
//                 if (distance < position_merge_threshold_) {
//                     // 合并两个锥桶（保留检测次数更多的）
//                     if (cone_i.detection_count >= cone_j.detection_count) {
//                         // 合并到cone_i
//                         cone_i.position.x = (cone_i.position.x + cone_j.position.x) / 2.0;
//                         cone_i.position.y = (cone_i.position.y + cone_j.position.y) / 2.0;
//                         cone_i.detection_count += cone_j.detection_count;
//                         ids_to_remove.push_back(ids_to_keep[j]);
//                         ROS_DEBUG("Clustering: Merged cone %zu into %zu", ids_to_keep[j], ids_to_keep[i]);
//                     } else {
//                         // 合并到cone_j
//                         cone_j.position.x = (cone_i.position.x + cone_j.position.x) / 2.0;
//                         cone_j.position.y = (cone_i.position.y + cone_j.position.y) / 2.0;
//                         cone_j.detection_count += cone_i.detection_count;
//                         ids_to_remove.push_back(ids_to_keep[i]);
//                         ROS_DEBUG("Clustering: Merged cone %zu into %zu", ids_to_keep[i], ids_to_keep[j]);
//                         break; // 跳出内层循环
//                     }
//                 }
//             }
//         }
        
//         // 删除被合并的锥桶
//         for (size_t id : ids_to_remove) {
//             global_cones_.erase(id);
//         }
        
//         ROS_DEBUG("Clustering complete. Removed %zu cones", ids_to_remove.size());
//     }

//     void publishVehiclePose() {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped.header.stamp = ros::Time::now();
//         transformStamped.header.frame_id = "map";
//         transformStamped.child_frame_id = "base_link";
        
//         transformStamped.transform.translation.x = current_pose_.position.x;
//         transformStamped.transform.translation.y = current_pose_.position.y;
//         transformStamped.transform.translation.z = 0.0;
//         transformStamped.transform.rotation = current_pose_.orientation;
        
//         try {
//             tf_broadcaster_.sendTransform(transformStamped);
//         } catch (const std::exception& e) {
//             ROS_ERROR("TF broadcast error: %s", e.what());
//         }
//     }

//     // ROS相关
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_cones_;
//     ros::Subscriber sub_slam_;
//     ros::Publisher pub_markers_;
//     ros::Publisher pub_global_cones_;
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
//     tf2_ros::TransformBroadcaster tf_broadcaster_;
    
//     // 锥桶数据
//     std::unordered_map<size_t, ConeData> global_cones_;
//     geometry_msgs::Pose current_pose_;
//     ros::Time last_publish_time_;
//     ros::Time last_slam_update_time_;
//     ros::Time last_cone_update_time_;
    
//     // 参数
//     double cone_lifetime_;
//     double max_distance_;
//     double lidar_offset_x_;
//     double position_merge_threshold_;  // 位置合并阈值
//     double position_smoothing_factor_; // 位置平滑因子
    
//     // 锥桶尺寸
//     geometry_msgs::Vector3 red_cone_scale_;
//     geometry_msgs::Vector3 blue_cone_scale_;
//     geometry_msgs::Vector3 yellow_cone_scale_;
    
//     // 变换矩阵
//     Eigen::Affine3d vehicle_to_map_transform_;
// };

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "cone_mapper");
//     ConeMapper mapper;
//     ros::spin();
//     return 0;
// }