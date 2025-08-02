/**
 * @file box_controller.cpp
 * @brief Box机器人横向误差控制器 - 基于前视距离的PID控制
 * 适用于0关节1连杆6自由度的盒子机器人
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
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
    ros::Subscriber model_states_sub_;   // 订阅Gazebo模型状态（获取当前位置和方向）
    
    // 发布器
    ros::Publisher cmd_vel_pub_;         // 发布控制命令
    ros::Publisher control_viz_pub_;     // 发布控制状态可视化
    
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
    
    // 机器人轨迹点（实际走过的路径）
    std::vector<geometry_msgs::Point> robot_trajectory_;
    
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
        planning_sub_ = nh_.subscribe("/line_points", 10, 
                                     &BoxController::planningCallback, this);
        
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                         &BoxController::modelStatesCallback, this);
        
        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        control_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/control_viz", 10);
        
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
     * @brief 接收Gazebo模型状态信息
     */
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // 查找box_robot模型
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "box_robot") {
                // 更新当前位置
                current_position_ = msg->pose[i].position;
                
                // 添加当前位置到轨迹中（每隔一定距离记录一次）
                if (robot_trajectory_.empty() || 
                    sqrt(pow(current_position_.x - robot_trajectory_.back().x, 2) + 
                         pow(current_position_.y - robot_trajectory_.back().y, 2)) > 0.1) {
                    robot_trajectory_.push_back(current_position_);
                    
                    // 限制轨迹点数量
                    if (robot_trajectory_.size() > 1000) {
                        robot_trajectory_.erase(robot_trajectory_.begin());
                    }
                }
                
                // 提取当前朝向角
                tf::Quaternion q(
                    msg->pose[i].orientation.x,
                    msg->pose[i].orientation.y,
                    msg->pose[i].orientation.z,
                    msg->pose[i].orientation.w
                );
                tf::Matrix3x3 m(q);
                double roll, pitch;
                m.getRPY(roll, pitch, current_yaw_);
                
                // 计算横向误差并执行PID控制
                if (!planning_points_.empty()) {
                    calculateLateralError();
                    pidControl();
                }
                break;
            }
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
        double dt = (current_time - last_time_).toSec();//ROS时间格式转换为秒
        
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
        
        // 发布控制命令（适用于3自由度盒子机器人）
        // 注意：planar_move插件工作在机器人本体坐标系下
        // 需要将world坐标系下的运动转换为机器人坐标系
        geometry_msgs::Twist cmd;
        
        // 将world坐标系下的恒定线速度转换到机器人坐标系
        // world坐标系下期望沿x轴正方向运动
        double world_vx = constant_linear_vel_;  // world坐标系下的x方向速度
        double world_vy = 0.0;                   // world坐标系下的y方向速度
        
        // 转换到机器人本体坐标系
        cmd.linear.x = world_vx * cos(current_yaw_) + world_vy * sin(current_yaw_);
        cmd.linear.y = -world_vx * sin(current_yaw_) + world_vy * cos(current_yaw_);
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
     * @brief 发布控制状态可视化（机器人轨迹线）
     */
    void publishControlLine()
    {
        visualization_msgs::MarkerArray marker_array;
        
        // 创建机器人轨迹线标记
        visualization_msgs::Marker trajectory_marker;
        trajectory_marker.header.frame_id = "world";
        trajectory_marker.header.stamp = ros::Time::now();
        trajectory_marker.ns = "robot_trajectory";
        trajectory_marker.id = 2;
        trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_marker.action = visualization_msgs::Marker::ADD;
        
        trajectory_marker.pose.orientation.w = 1.0;
        
        // 设置线条属性
        trajectory_marker.scale.x = 0.08;  // 线条宽度
        trajectory_marker.color.r = 0.0;
        trajectory_marker.color.g = 0.0;
        trajectory_marker.color.b = 1.0;   // 蓝色
        trajectory_marker.color.a = 0.8;   // 稍微透明
        
        // 添加机器人实际走过的轨迹点
        for (const auto& point : robot_trajectory_) {
            trajectory_marker.points.push_back(point);
        }
        
        // 只有当有足够的轨迹点时才显示轨迹线
        if (robot_trajectory_.size() > 1) {
            marker_array.markers.push_back(trajectory_marker);
        }
        
        // 发布可视化消息
        control_viz_pub_.publish(marker_array);
    }
    
    /**
     * @brief 运行控制器
     */
    void run()
    {
        ROS_INFO("Box Lateral Controller running...");
        ROS_INFO("Waiting for planning and model_states messages...");
        ROS_INFO("Publishing cmd_vel and control_viz topics...");
        
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