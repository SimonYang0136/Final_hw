#include<iostream>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/Point.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
// 根据前几个锥桶计算得到的中线的参数，用于后续判断左右锥桶
#define K -1.333
#define B -12.65
// 默认车辆坐标（可用于后续起点修正等）
geometry_msgs::Point g_vehicle_pose = [](){
    geometry_msgs::Point p;
    p.x = -7.135;
    p.y = -3.405; 
    p.z = 0.0;
    return p;
}();

// 速度规划：根据曲率或点间距离调整速度，弯道慢，直线快
std::vector<double> plan_speed(const std::vector<geometry_msgs::Point>& mid_points)
{
    std::vector<double> speeds(mid_points.size(), 2.0); // 默认速度2.0
    if(mid_points.size() < 3) 
        return speeds;
    for(size_t i=1; i+1<mid_points.size(); ++i) 
    {
        const auto& p_prev = mid_points[i-1];
        const auto& p = mid_points[i];
        const auto& p_next = mid_points[i+1];
        // 计算向量
        double dx1 = p.x - p_prev.x;
        double dy1 = p.y - p_prev.y;
        double dx2 = p_next.x - p.x;
        double dy2 = p_next.y - p.y;
        // 计算夹角
        double dot = dx1*dx2 + dy1*dy2;
        double norm1 = sqrt(dx1*dx1 + dy1*dy1);
        double norm2 = sqrt(dx2*dx2 + dy2*dy2);
        double angle = acos(std::max(-1.0, std::min(1.0, dot/(norm1*norm2+1e-6))));
        // 根据弯道角度调整速度，弯道越急速度越低
        if(angle > 0.5) 
            speeds[i] = 1.0; // 急弯
        else if(angle > 0.2) 
            speeds[i] = 1.5; // 普通弯
        else 
            speeds[i] = 2.5; // 直线
    }
    return speeds;
}

// 创建中心线
visualization_msgs::Marker create_center_line(const std::vector<geometry_msgs::Point>& cone_left, const std::vector<geometry_msgs::Point>& cone_right,ros::Publisher& pub ) 
{
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "planning line";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.2; // 规划线宽度
    line.color.r = 1.0f;
    line.color.a = 1.0f;
    // 创建一个空的mid_points容器用于存储中点坐标
    std::vector<geometry_msgs::Point> mid_points;
    // 默认车辆起点
    mid_points.push_back(g_vehicle_pose);
    // 按x坐标对左右锥桶排序，确保配对顺序一致
    std::vector<geometry_msgs::Point> left_sorted = cone_left;
    std::vector<geometry_msgs::Point> right_sorted = cone_right;
    std::sort(left_sorted.begin(), left_sorted.end(), [](const geometry_msgs::Point& a, const geometry_msgs::Point& b){ return a.x < b.x; });
    std::sort(right_sorted.begin(), right_sorted.end(), [](const geometry_msgs::Point& a, const geometry_msgs::Point& b){ return a.x < b.x; });
    size_t n = std::min(left_sorted.size(), right_sorted.size());
    // 至少需要两个点才能配对
    if (n < 2) 
    {
        line.action = visualization_msgs::Marker::DELETE;
        return line;
    }
    //连接中点
    for(size_t i=0; i<n; i++) 
    {
        geometry_msgs::Point mid;
        geometry_msgs::Point mid2;
        mid.x = (left_sorted[i].x + right_sorted[i].x) / 2.0;
        mid.y = (left_sorted[i].y + right_sorted[i].y) / 2.0;
        mid.z = 0;
        mid_points.push_back(mid);
        if(i==n-1)//防止越界
            break;
        mid2.x = (left_sorted[i].x + right_sorted[i+1].x) / 2.0;
        mid2.y = (left_sorted[i].y + right_sorted[i+1].y) / 2.0;
        mid2.z = 0;
        mid_points.push_back(mid2);
    }
    for (size_t i=0; i<mid_points.size(); ++i)
        pub.publish(mid_points[i]);
    // 速度规划
    std::vector<double> speeds = plan_speed(mid_points);
    line.points = mid_points;
    return line;
}

void cones_address(const visualization_msgs::MarkerArray::ConstPtr& cones, ros::Publisher& pub, ros::Publisher& pub2)
{
    // 先收集原始锥桶坐标
    std::vector<geometry_msgs::Point> all_cones;
    for(const auto& marker : cones->markers) 
        all_cones.push_back(marker.pose.position);
    // 划分左右锥桶
    std::vector<geometry_msgs::Point> cone_left, cone_right;
    for(const auto& p : all_cones) 
    {
        if (std::abs(K*p.x-p.y + B)/(sqrt(K*K+B*B))>0.5)
            continue;
        if (K*p.x-p.y + B > 0)
            cone_left.push_back(p);
        else
            cone_right.push_back(p);
    }
    //将中点连线
    auto line = create_center_line(cone_left, cone_right ,pub2);
    pub.publish(line);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/planning", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Point>("/line_points",10);
    //用lambda表达式获取发布方，这样就不用定义为全局变量
    auto callback = [&pub,&pub2](const visualization_msgs::MarkerArray::ConstPtr& cones) 
    {
        cones_address(cones, pub,pub2);
    };
    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/mapping/cones", 10, callback);
    //广播静态变换，将map中的起点线处的两个锥桶的中点变换为world中的原点，跑道方向为world的x轴
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";   // 父坐标系
    transform.child_frame_id = "map";      // 子坐标系
    // map 的原点在 world 中的位置
    transform.transform.translation.x = 7.135;
    transform.transform.translation.y = 3.405;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.9078);  // roll, pitch, yaw（单位：弧度）
    quat.normalize(); 
    double yaw = 0.9078;
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    // 原始偏移量（未旋转前）
    double dx = 7.135;
    double dy = 3.405;
    // 把偏移量旋转 -yaw 后再取负，使 (-7.135, -3.405) 落在 world 原点
    transform.transform.translation.x = -(dx * cos_yaw + dy * sin_yaw);
    transform.transform.translation.y = -(-dx * sin_yaw + dy * cos_yaw);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transform);
    ros::spin();
    return 0;
}