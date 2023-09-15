//1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
//保存乌龟名称
std::string turtle_name;
int flag_tf = 0;
void doPose(const nav_msgs::Odometry::ConstPtr& odom){
    //  6-1.创建 TF 广播器 ---------------------------------------- 注意 static
    static tf2_ros::TransformBroadcaster broadcaster;
    //  6-2.将 pose 信息转换成 TransFormStamped
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "odom";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "base_footprint";
    tfs.transform.translation.x = odom->pose.pose.position.x;
    tfs.transform.translation.y = odom->pose.pose.position.y;
    tfs.transform.translation.z = 0.0;
    tfs.transform.rotation.x = odom->pose.pose.orientation.x;
    tfs.transform.rotation.y = odom->pose.pose.orientation.y;
    tfs.transform.rotation.z = odom->pose.pose.orientation.z;
    tfs.transform.rotation.w = odom->pose.pose.orientation.w;
    //  6-3.发布
    broadcaster.sendTransform(tfs);
    flag_tf = 1;
    // std::cout<<"11111111"<<std::endl;
} 

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"pub_tf");
    // 4.创建 ros 句柄
    ros::NodeHandle nh;
    // 5.创建订阅对象
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odom",1000,doPose);
    ros::Rate rate(30);
    //     6.回调函数处理订阅的 pose 信息
    //         6-1.创建 TF 广播器
    //         6-2.将 pose 信息转换成 TransFormStamped
    //         6-3.发布
    // 7.spin
    ros::spin();
    return 0;
}
