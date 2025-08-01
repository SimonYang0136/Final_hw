#include<iostream>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/Point.h>
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

// 创建中心线
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
    // 先收集原始锥桶坐标
    std::vector<geometry_msgs::Point> all_cones;
    for(const auto& marker : cones->markers) {
        all_cones.push_back(marker.pose.position);
    }
    // 分组
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
    // 调试输出分组后锥桶数量
    ROS_INFO("Left cones: %lu, Right cones: %lu", cone_left.size(), cone_right.size());
    //将中点连线
    auto line = create_center_line(cone_left, cone_right);
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