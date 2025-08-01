#include<iostream>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>

const double cos_theta = (3.04)/(sqrt(pow(3.04,2)+pow(-3.705,2)));
const double sin_theta = (-3.705)/(sqrt(pow(3.04,2)+pow(-3.705,2)));
// 默认车辆坐标（可用于后续起点修正等）
geometry_msgs::Point g_vehicle_pose = [](){
    geometry_msgs::Point p;
    p.x = -7.135;
    p.y = -3.405; 
    p.z = 0.0;
    return p;
}();

// 创建中心线
int n=0;
visualization_msgs::Marker create_center_line(const std::vector<geometry_msgs::Point>& cone_left, const std::vector<geometry_msgs::Point>& cone_right) 
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
    // 创建一个空的mid_points向量用于存储中点坐标
    std::vector<geometry_msgs::Point> mid_points;
    // 逆旋转参数（与cones_address一致）
    // 车辆起点直接用原始坐标
    mid_points.push_back(g_vehicle_pose);
    // 按x坐标排序，确保配对顺序一致
    std::vector<geometry_msgs::Point> left_sorted = cone_left;
    std::vector<geometry_msgs::Point> right_sorted = cone_right;
    std::sort(left_sorted.begin(), left_sorted.end(), [](const geometry_msgs::Point& a, const geometry_msgs::Point& b){ return a.x < b.x; });
    std::sort(right_sorted.begin(), right_sorted.end(), [](const geometry_msgs::Point& a, const geometry_msgs::Point& b){ return a.x < b.x; });
    size_t n = std::min(left_sorted.size(), right_sorted.size());
    for(size_t i=0; i<n; i++)
    {
        geometry_msgs::Point mid;
        mid.x = (left_sorted[i].x + right_sorted[i].x) / 2.0;
        mid.y = (left_sorted[i].y + right_sorted[i].y) / 2.0;
        mid.z = 0;
        // 逆旋转回原始坐标系（注意公式）
        geometry_msgs::Point mid_rot;
        mid_rot.x = (mid.x*cos_theta - mid.y*sin_theta)/(pow(cos_theta,2)-pow(sin_theta,2));
        mid_rot.y = (mid.x*sin_theta - mid.y*cos_theta)/(pow(sin_theta,2)-pow(cos_theta,2));
        mid_rot.z = 0;
        mid_points.push_back(mid_rot);
    }
    // 至少需要两个点，否则不返回有效Marker
    if(mid_points.size() < 2) {
        line.action = visualization_msgs::Marker::DELETE;
        return line;
    }
    line.points = mid_points;
    return line;
}

void cones_address(const visualization_msgs::MarkerArray::ConstPtr& cones, ros::Publisher& pub)
{
    std::vector<geometry_msgs::Point> cone_left,cone_right;
    
    for(const auto& marker : cones->markers) 
    {
        geometry_msgs::Point p;
        //获取锥桶坐标信息
        p.x=marker.pose.position.x*cos_theta-marker.pose.position.y*sin_theta;
        p.y=marker.pose.position.x*sin_theta+marker.pose.position.y*cos_theta;
        p.z=0;
        if (p.y<0)
            cone_left.push_back(p);
        else
            cone_right.push_back(p);
    }
    // 调试输出分组后锥桶数量
    ROS_INFO("Left cones: %lu, Right cones: %lu", cone_left.size(), cone_right.size());
    //将中点连线
    auto line = create_center_line(cone_left,cone_right);
    pub.publish(line);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/planning", 10);
    //用lambda表达式获取发布方，这样就不用定义为全局变量
    auto callback = [&pub](const visualization_msgs::MarkerArray::ConstPtr& cones) 
    {
        cones_address(cones, pub);
    };
    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/mapping/cones", 10, callback);
    ros::spin();
    return 0;
}