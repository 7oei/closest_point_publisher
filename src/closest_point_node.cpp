#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    double distance_range_max = 2;      //追従できる最大距離
    double distanec_range_min = 0.5;    //停止する距離

    double hight_range_max = 0.2;      //検出する最大高さ
    double hight_range_min = -0.1;     //検出する最小高さ

    double angle_range = 45 / 180 * 3.141592; //左右角度どのくらいまで検出するか

    int ugoku = 1;          //動ける時１、止まる時０
    double tra_max = 1;     //最大機体並進速度
    double rot_max = 1.8;   //最大機体角速度

    double tra_gain = 1;    //並進速度比例ゲイン
    double rot_gain = 0.2;  //角速度比例ゲイン

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    if(cloud->points.size() == 0)
    {
        ROS_WARN("Received empty point cloud");
        return;
    }

    ///
    double min_distance = std::numeric_limits<double>::max();   //範囲内で一番近い点
    double min_dis_angle = std::numeric_limits<double>::max();  //範囲内で一番近い点と原点を結んだ線が座標系と成す角度
    pcl::PointXYZ closest_point;
    
    //点群を絞り込んで一番近い範囲内の点を見つける処理
    for(const auto& point : cloud->points)
    {
        if(point.z < hight_range_max && point.z > hight_range_min)  //高さ範囲で絞り込む
        {
            if(point.x * point.x + point.y * point.y < distanec_range_min) //近すぎるものあったら止まるだけなので処理終了
            {
                ugoku = 0;
                break;
            }
            if(point.x * point.x  + point.y * point.y < distance_range_max) //円の範囲内に入らないものは処理しない
            {
                double angle = atan2(point.y, point.x);
                if(angle > -angle_range && angle < angle_range)//角度範囲内でなければ処理しない
                {
                    double distance = std::sqrt(point.x * point.x + point.y * point.y); //点との距離
                    if(distance < min_distance)
                    {
                        min_dis_angle = angle;
                        min_distance = distance;
                        closest_point = point;

                        ugoku = 1;
                    }
                }
            }
        }
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = input->header.frame_id;
    pose.pose.position.x = closest_point.x;
    pose.pose.position.y = closest_point.y;
    pose.pose.position.z = closest_point.z;

    double velocity_tra = 0;    //機体並進速度指令値
    double velocity_rot = 0;    //機体角速度指令値
    if(ugoku == 0) return;

    //指定した範囲で正規化し、最大速度とゲインをかける
    velocity_tra = tra_gain * tra_max * (min_distance - distanec_range_min) / (distance_range_max - distanec_range_min);
    velocity_rot = rot_gain * rot_max * min_dis_angle / angle_range;

    
    ///

    pub.publish(pose);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "closest_point_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>("/master_pose", 1);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, velodyneCallback);

    ros::spin();

    return 0;
}