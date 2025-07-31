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
    p.x = -3.0;
    p.y = 0.2; 
    p.z = 0.0;
    return p;
}();

// 创建中心线
int n=0;
visualization_msgs::Marker create_center_line(const std::vector<geometry_msgs::Point>& left_cones, const std::vector<geometry_msgs::Point>& right_cones) 
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
    // 遍历左边的锥桶
    for(const auto& left : left_cones)
    {
        // 初始化最小距离为1e6
        double min_dist = 1e6;
        // 初始化最小索引为-1
        int min_idx = -1;
        // 遍历右边的锥桶
        for(size_t j = 0; j < right_cones.size(); j++) 
        {
            // 计算左锥形和右锥形的距离
            double dx = left.x - right_cones[j].x;
            double dy = left.y - right_cones[j].y;
            double d = std::sqrt(dx*dx + dy*dy);
            // 挑选出距离最近的右锥桶，与当前左锥桶相对应
            if(d < min_dist) 
            {
                min_dist = d;
                min_idx = j;
            }
        }
        //计算中点，并将其添加到mid_points向量中
        if(min_idx >= 0) 
        {
            geometry_msgs::Point mid;
            mid.x = (left.x + right_cones[min_idx].x) / 2.0;
            mid.y = (left.y + right_cones[min_idx].y) / 2.0;
            mid.z = (left.z + right_cones[min_idx].z) / 2.0;
            mid_points.push_back(mid);
        }
    } 
    // 对mid_points向量进行排序，后续依次连成线
    std::sort(mid_points.begin(), mid_points.end(), [](const geometry_msgs::Point& a, const geometry_msgs::Point& b){ return a.x < b.x; });
    // 以车辆坐标为起点
    if (n<50)
    {
        line.points.push_back(g_vehicle_pose);
        n++;
    }
    for(const auto& pt : mid_points) line.points.push_back(pt);
    // 返回Marker对象
    return line;
}

void cones_address(const visualization_msgs::MarkerArray::ConstPtr& cones, ros::Publisher& pub)
{
    //定义一个横向的最大范围用于排除跑道外的误检点
    constexpr double max = 3.0; 
    //定义一个空的left_cones和right_cones向量用于存储左右锥桶的坐标
    std::vector<geometry_msgs::Point> left_cones, right_cones;
    //预留内存空间
    left_cones.reserve(cones->markers.size());
    right_cones.reserve(cones->markers.size());
    //遍历所有锥桶，将左右锥桶分别存储到left_cones和right_cones向量中
    for(const auto& marker : cones->markers) 
    {
        //获取锥桶坐标信息
        const auto& p = marker.pose.position;
        //原点左边的是左锥桶，右边的是右锥桶......
        if(std::abs(p.y) > max) continue; //筛除跑道外的点
        if(p.y > 0) left_cones.push_back(p);
        else right_cones.push_back(p);
    }
    //将中点连线
    auto line = create_center_line(left_cones, right_cones);
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