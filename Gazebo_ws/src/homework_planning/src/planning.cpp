#include<iostream>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>


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
visualization_msgs::Marker create_center_line(const std::vector<geometry_msgs::Point>& cone) 
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
    mid_points.push_back(g_vehicle_pose);
    for(size_t i=0; i<cone.size()-1; i++)
    {
        // 计算左右锥桶的中点
        geometry_msgs::Point mid;
        mid.x = (cone[i].x + cone[i+1].x) / 2.0;
        mid.y = (cone[i].y + cone[i+1].y) / 2.0;
        mid.z=0;
        mid_points.push_back(mid);
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
    //定义一个横向的最大范围用于排除跑道外的误检点
    constexpr double max = 3.0; 
    //定义一个空的left_cones和right_cones向量用于存储左右锥桶的坐标
    std::vector<geometry_msgs::Point> cone;
    //遍历所有锥桶，将左右锥桶分别存储到left_cones和right_cones向量中
    for(const auto& marker : cones->markers) 
    {
        //获取锥桶坐标信息
        const auto& p = marker.pose.position;
        cone.push_back(p);
    }
    //将中点连线
    auto line = create_center_line(cone);
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