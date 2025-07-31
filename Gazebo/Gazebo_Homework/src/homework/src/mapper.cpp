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
#include <fsd_common_msgs/ConeDetections.h>
#include <fsd_common_msgs/CarState.h>

class ConeMapper {
public:
    ConeMapper() : tf_listener_(tf_buffer_) {
        // 订阅锥桶检测和SLAM状态
        sub_cones_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &ConeMapper::coneCallback, this);
        sub_slam_ = nh_.subscribe("/estimation/slam/state", 10, &ConeMapper::slamCallback, this);
        
        // 发布建图结果
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/mapping/cones", 10);
        
        // 雷达到惯导的固定变换 (2.4m)
        lidar_to_imu_.transform.translation.x = 2.4;
        lidar_to_imu_.transform.translation.y = 0.0;
        lidar_to_imu_.transform.translation.z = 0.0;
        lidar_to_imu_.transform.rotation.w = 1.0;
        lidar_to_imu_.header.frame_id = "base_link";
        lidar_to_imu_.child_frame_id = "lidar";

        // 初始化锥桶尺寸
        red_cone_scale_.x = 0.2;   // 底面直径
        red_cone_scale_.y = 0.2;
        red_cone_scale_.z = 0.3;   // 高度
        
        blue_cone_scale_ = red_cone_scale_;  // 蓝色锥桶使用相同尺寸
        
        ROS_INFO("ConeMapper initialized");
    }

    void slamCallback(const fsd_common_msgs::CarState::ConstPtr& msg) {
        // 更新车辆位姿
        current_pose_.position.x = msg->car_state.x;
        current_pose_.position.y = msg->car_state.y;
        
        // 创建四元数表示偏航角
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->car_state.theta);
        current_pose_.orientation = tf2::toMsg(q);
        
        // 发布车辆位姿
        publishVehiclePose();
    }

    void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        
        // 添加删除所有旧标记的指令
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        delete_all.ns = "cones";
        marker_array.markers.push_back(delete_all);
        
        // 添加测试标记
        // visualization_msgs::Marker test_marker;
        // test_marker.header.frame_id = "map";
        // test_marker.header.stamp = ros::Time::now();
        // test_marker.ns = "test";
        // test_marker.id = 9999;
        // test_marker.type = visualization_msgs::Marker::SPHERE;
        // test_marker.action = visualization_msgs::Marker::ADD;
        // test_marker.pose.position.x = 0;
        // test_marker.pose.position.y = 0;
        // test_marker.pose.position.z = 0;
        // test_marker.scale.x = 0.5;
        // test_marker.scale.y = 0.5;
        // test_marker.scale.z = 0.5;
        // test_marker.color.r = 0.0;
        // test_marker.color.g = 1.0;
        // test_marker.color.b = 0.0;
        // test_marker.color.a = 1.0;
        // test_marker.lifetime = ros::Duration(10.0);
        // marker_array.markers.push_back(test_marker);
        
        // 遍历所有检测到的锥桶
        for (size_t i = 0; i < msg->cone_detections.size(); ++i) {
            const auto& cone = msg->cone_detections[i];
            
            // 创建Lidar坐标系中的位姿
            geometry_msgs::PoseStamped cone_in_lidar;
            cone_in_lidar.header = msg->header;
            cone_in_lidar.pose.position = cone.position;
            cone_in_lidar.pose.orientation.w = 1.0;
            //直接进行坐标变换而非下面的坐标系变换
            geometry_msgs::PoseStamped cone_in_world;
            cone_in_world.pose.position.x=cone_in_lidar.pose.position.x-2.4;
            cone_in_world.pose.position.y=cone_in_lidar.pose.position.y;
            cone_in_world.pose.orientation.w = 1.0;
            try {
                // // 1. 转换到base_link坐标系
                // geometry_msgs::PoseStamped cone_in_base;
                // tf_buffer_.transform(cone_in_lidar, cone_in_base, "base_link"); 
                                     
                
                // // 2. 应用雷达到惯导的固定变换
                // geometry_msgs::PoseStamped cone_in_imu;
                // tf2::doTransform(cone_in_base, cone_in_imu, lidar_to_imu_);
                
                // // 3. 转换到世界坐标系
                // geometry_msgs::PoseStamped cone_in_world;
                // tf_buffer_.transform(cone_in_imu, cone_in_world, "map");
                                     
                
                // 创建Marker对象
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "cones";
                marker.id = i;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = cone_in_world.pose;
                
                // 根据颜色设置属性,事实上，这个bag就没给颜色信息，所以这里默认的是黄色
                if ( cone.color.data == "r") {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.8;  // 红色，带透明度
                    marker.scale = red_cone_scale_;
                } else if (cone.color.data == "b") {
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 0.8;  // 蓝色
                    marker.scale = blue_cone_scale_;
                } else {
                    // 未知颜色默认为黄色(全是黄色)
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.8;
                    marker.scale = red_cone_scale_;
                }
                
                marker.lifetime = ros::Duration(30.0);  // 保留时间，我试图做成永久的标记但失败了
                marker_array.markers.push_back(marker);
                
            }
            //报错用的（一般用不到，可以忽略）
            catch (tf2::TransformException &ex) {
                ROS_WARN("Transform failure for cone %zu: %s", i, ex.what());
            }
        }
        //发布标记信息
        pub_markers_.publish(marker_array);
    }

private:
    void publishVehiclePose() {
        geometry_msgs::TransformStamped transformStamped;
        //设置标记的基本信息
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        
        transformStamped.transform.translation.x = current_pose_.position.x;
        transformStamped.transform.translation.y = current_pose_.position.y;
        //由于bag中的位姿信息没有z，直接设成0了
        transformStamped.transform.translation.z = 0.0;
        
        transformStamped.transform.rotation = current_pose_.orientation;
        
    }
    //需要用到的变量
    ros::NodeHandle nh_;
    ros::Subscriber sub_cones_;
    ros::Subscriber sub_slam_;
    ros::Publisher pub_markers_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::TransformStamped lidar_to_imu_;
    geometry_msgs::Vector3 red_cone_scale_, blue_cone_scale_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cone_mapper");
    ConeMapper mapper;
    ros::spin();
    return 0;
}