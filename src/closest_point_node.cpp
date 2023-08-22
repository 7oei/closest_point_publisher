#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    if(cloud->points.size() == 0)
    {
        ROS_WARN("Received empty point cloud");
        return;
    }

    double min_distance = std::numeric_limits<double>::max();
    pcl::PointXYZ closest_point;

    for(const auto& point : cloud->points)
    {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if(distance < min_distance)
        {
            min_distance = distance;
            closest_point = point;
        }
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = input->header.frame_id;
    pose.pose.position.x = closest_point.x;
    pose.pose.position.y = closest_point.y;
    pose.pose.position.z = closest_point.z;

    pub.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "closest_point_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>("closest_point_pose", 1);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, velodyneCallback);

    ros::spin();

    return 0;
}