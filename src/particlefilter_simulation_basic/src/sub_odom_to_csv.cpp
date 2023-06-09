// odomをsubしてきて、x,yだけcsvに出力するプログラム
#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>



std::ofstream ofs("odom_subscribe.csv");
geometry_msgs::Quaternion robot_r;

double robot_x = 0.0;
double robot_y = 0.0;

// odomの位置(ｘ,y,theta)をsub
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;

}

// クォータニオンをオイラーに変換
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  
}

// メインの処理
int main(int argc, char **argv)
{

    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    // odomのsubscribe
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, odomCallback);
    ros::Rate rate(10.0);

    while (ros::ok())
    {
        ros::spinOnce();
        double roll, pitch, yaw;
        geometry_quat_to_rpy(roll, pitch, yaw, robot_r); //yaw角に変換
        // 実際の自己位置をcsvに出力
        std::cout << robot_x << ", " << robot_y << ", " << yaw << std::endl;
        // csvにx,y,yawを保存
        ofs << robot_x << ", " << robot_y << ", " << yaw << std::endl;

    }

    rate.sleep();

}