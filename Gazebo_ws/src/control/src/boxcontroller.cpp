/**
 * @file box_controller.cpp
 * @brief Box机器人横向误差控制器 - 基于前视距离的PID控制
 * 适用于0关节1连杆6自由度的盒子机器人
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <math.h>

class BoxController
{
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    ros::Subscriber planning_sub_;       // 订阅规划路径点
    ros::Subscriber odom_sub_;           // 订阅里程计（获取当前位置和方向）
    
    // 发布器
    ros::Publisher cmd_vel_pub_;         // 发布控制命令
    ros::Publisher control_line_pub_;    // 发布可视化路径
    
    // 控制参数
    double constant_linear_vel_;         // 恒定线速度
    double lookahead_distance_;          // 前视距离
    double lateral_error_;               // 横向误差
    
    // PID参数
    double kp_, ki_, kd_;                // PID系数
    double error_sum_;                   // 积分项
    double last_error_;                  // 上次误差
    ros::Time last_time_;               // 上次时间
    
    // 高级参数
    double max_angular_vel_;             // 最大角速度限制
    double integral_limit_;              // 积分限幅值
    int path_points_limit_;              // 路径点数量限制
    int debug_frequency_;                // 调试信息打印频率
    
    // 路径点
    std::vector<geometry_msgs::Point> planning_points_;
    
    // 当前状态
    geometry_msgs::Point current_position_;
    double current_yaw_;

public:
    BoxController() : 
        constant_linear_vel_(0.5),
        lookahead_distance_(1.0),
        lateral_error_(0.0),
        kp_(1.0),
        ki_(0.1),
        kd_(0.05),
        error_sum_(0.0),
        last_error_(0.0),
        current_yaw_(0.0),
        max_angular_vel_(2.0),
        integral_limit_(1.0),
        path_points_limit_(200),
        debug_frequency_(20)
    {
        // 从参数服务器读取参数
        nh_.param("constant_linear_velocity", constant_linear_vel_, 0.5);
        nh_.param("lookahead_distance", lookahead_distance_, 1.0);
        nh_.param("pid_kp", kp_, 1.0);
        nh_.param("pid_ki", ki_, 0.1);
        nh_.param("pid_kd", kd_, 0.05);
        nh_.param("max_angular_velocity", max_angular_vel_, 2.0);
        nh_.param("integral_limit", integral_limit_, 1.0);
        nh_.param("path_points_limit", path_points_limit_, 200);
        nh_.param("debug_frequency", debug_frequency_, 20);
        
        // 初始化订阅器
        planning_sub_ = nh_.subscribe("/planning", 10, 
                                     &BoxController::planningCallback, this);
        
        odom_sub_ = nh_.subscribe("/odom", 10,
                                 &BoxController::odomCallback, this);
        
        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        control_line_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/control_line", 10);
        
        last_time_ = ros::Time::now();
        
        ROS_INFO("=== Box Robot Lateral Controller Started ===");
        ROS_INFO("Constant linear velocity: %.3f m/s", constant_linear_vel_);
        ROS_INFO("Lookahead distance: %.3f m", lookahead_distance_);
        ROS_INFO("PID gains - Kp:%.3f, Ki:%.3f, Kd:%.3f", kp_, ki_, kd_);
        ROS_INFO("Max angular velocity: %.3f rad/s", max_angular_vel_);
        ROS_INFO("Integral limit: %.3f", integral_limit_);
        ROS_INFO("Path points limit: %d", path_points_limit_);
    }
    
    /**
     * @brief 接收规划路径点
     */
    void planningCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        // 添加新的规划点
        planning_points_.push_back(*msg);
        
        // 限制路径点数量（使用参数服务器的值）
        if (planning_points_.size() > static_cast<size_t>(path_points_limit_)) {
            planning_points_.erase(planning_points_.begin());
        }
        
        // 发布可视化路径
        publishControlLine();
        
        ROS_DEBUG("Received planning point: (%.3f, %.3f, %.3f)", 
                 msg->x, msg->y, msg->z);
    }
    
    /**
     * @brief 接收里程计信息
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 更新当前位置
        current_position_ = msg->pose.pose.position;
        
        // 提取当前朝向角
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        // 计算横向误差并执行PID控制
        if (!planning_points_.empty()) {
            calculateLateralError();
            pidControl();
        }
    }
    
    /**
     * @brief 计算横向误差
     */
    void calculateLateralError()
    {
        if (planning_points_.empty()) {
            lateral_error_ = 0.0;
            return;
        }
        
        // 计算前视点
        geometry_msgs::Point lookahead_point;
        bool found_lookahead = false;
        
        // 遍历规划点，找到距离当前位置前视距离的点
        for (size_t i = 0; i < planning_points_.size(); ++i) {
            double distance = sqrt(
                pow(planning_points_[i].x - current_position_.x, 2) +
                pow(planning_points_[i].y - current_position_.y, 2)
            );
            
            // 找到距离前视距离最近的点
            if (distance >= lookahead_distance_) {
                lookahead_point = planning_points_[i];
                found_lookahead = true;
                break;
            }
        }
        
        // 如果没找到足够远的点，使用最后一个点
        if (!found_lookahead && !planning_points_.empty()) {
            lookahead_point = planning_points_.back();
        }
        
        // 计算横向误差（相对于车体坐标系）
        // 将世界坐标系下的前视点转换到车体坐标系
        double dx = lookahead_point.x - current_position_.x;
        double dy = lookahead_point.y - current_position_.y;
        
        // 转换到车体坐标系（车头方向为x轴正方向）
        double local_x = dx * cos(current_yaw_) + dy * sin(current_yaw_);
        double local_y = -dx * sin(current_yaw_) + dy * cos(current_yaw_);
        
        // 横向误差就是车体坐标系下的y坐标
        lateral_error_ = local_y;
        
        ROS_DEBUG("Lookahead point: (%.3f, %.3f), Lateral error: %.3f", 
                 lookahead_point.x, lookahead_point.y, lateral_error_);
    }
    
    /**
     * @brief PID横向误差控制
     */
    void pidControl()
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        
        if (dt <= 0.0) return;  // 避免除零
        
        // 使用横向误差作为控制误差
        double error = lateral_error_;
        
        // PID计算
        error_sum_ += error * dt;                    // 积分项
        double error_diff = (error - last_error_) / dt;  // 微分项
        
        // 积分限幅（防止积分饱和）
        if (error_sum_ > integral_limit_) error_sum_ = integral_limit_;
        if (error_sum_ < -integral_limit_) error_sum_ = -integral_limit_;
        
        // PID输出（角速度）
        double angular_velocity = kp_ * error + ki_ * error_sum_ + kd_ * error_diff;
        
        // 角速度限幅
        if (angular_velocity > max_angular_vel_) angular_velocity = max_angular_vel_;
        if (angular_velocity < -max_angular_vel_) angular_velocity = -max_angular_vel_;
        
        // 发布控制命令（适用于6自由度盒子机器人）
        geometry_msgs::Twist cmd;
        cmd.linear.x = constant_linear_vel_;    // 恒定线速度
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = angular_velocity;       // 基于横向误差的角速度
        
        cmd_vel_pub_.publish(cmd);
        
        // 更新历史值
        last_error_ = error;
        last_time_ = current_time;
        
        // 调试信息
        static int count = 0;
        if (++count % debug_frequency_ == 0) {  // 使用参数服务器的调试频率
            ROS_INFO("Lateral Control - Error:%.3f, Angular:%.3f, Position:(%.2f,%.2f), Yaw:%.2f", 
                    lateral_error_, angular_velocity, 
                    current_position_.x, current_position_.y, current_yaw_);
        }
    }
    
    /**
     * @brief 发布控制路径可视化
     */
    void publishControlLine()
    {
        if (planning_points_.empty()) return;
        
        visualization_msgs::MarkerArray marker_array;
        
        // 创建规划路径线条标记
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "odom";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "planning_path";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        // 设置线条属性
        line_marker.scale.x = 0.05;  // 线宽
        line_marker.color.r = 0.0;   // 蓝色
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0;   // 不透明
        
        // 添加规划路径点
        for (const auto& point : planning_points_) {
            line_marker.points.push_back(point);
        }
        
        marker_array.markers.push_back(line_marker);
        
        // 创建前视点标记
        if (!planning_points_.empty()) {
            // 计算前视点
            geometry_msgs::Point lookahead_point;
            bool found_lookahead = false;
            
            for (const auto& point : planning_points_) {
                double distance = sqrt(
                    pow(point.x - current_position_.x, 2) +
                    pow(point.y - current_position_.y, 2)
                );
                
                if (distance >= lookahead_distance_) {
                    lookahead_point = point;
                    found_lookahead = true;
                    break;
                }
            }
            
            if (!found_lookahead) {
                lookahead_point = planning_points_.back();
            }
            
            // 前视点标记
            visualization_msgs::Marker lookahead_marker;
            lookahead_marker.header.frame_id = "odom";
            lookahead_marker.header.stamp = ros::Time::now();
            lookahead_marker.ns = "lookahead_point";
            lookahead_marker.id = 1;
            lookahead_marker.type = visualization_msgs::Marker::SPHERE;
            lookahead_marker.action = visualization_msgs::Marker::ADD;
            
            lookahead_marker.pose.position = lookahead_point;
            lookahead_marker.pose.orientation.w = 1.0;
            
            lookahead_marker.scale.x = 0.2;
            lookahead_marker.scale.y = 0.2;
            lookahead_marker.scale.z = 0.2;
            lookahead_marker.color.r = 1.0;  // 红色
            lookahead_marker.color.g = 0.0;
            lookahead_marker.color.b = 0.0;
            lookahead_marker.color.a = 1.0;
            
            marker_array.markers.push_back(lookahead_marker);
        }
        
        // 创建当前位置标记
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "robot_position";
        robot_marker.id = 2;
        robot_marker.type = visualization_msgs::Marker::ARROW;
        robot_marker.action = visualization_msgs::Marker::ADD;
        
        robot_marker.pose.position = current_position_;
        tf::Quaternion q = tf::createQuaternionFromYaw(current_yaw_);
        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();
        
        robot_marker.scale.x = 0.3;  // 箭头长度
        robot_marker.scale.y = 0.1;  // 箭头宽度
        robot_marker.scale.z = 0.1;  // 箭头高度
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 1.0;  // 绿色
        robot_marker.color.b = 0.0;
        robot_marker.color.a = 1.0;
        
        marker_array.markers.push_back(robot_marker);
        
        // 发布可视化消息
        control_line_pub_.publish(marker_array);
    }
    
    /**
     * @brief 运行控制器
     */
    void run()
    {
        ROS_INFO("Box Lateral Controller running...");
        ROS_INFO("Waiting for planning and odom messages...");
        ROS_INFO("Publishing cmd_vel and control_line topics...");
        
        ros::spin();  // 事件驱动
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_controller");
    
    BoxController controller;
    controller.run();
    
    return 0;
}