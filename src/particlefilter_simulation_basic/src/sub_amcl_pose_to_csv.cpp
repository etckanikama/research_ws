// amclを回したときに出力されるamcl_poseをsubしてきて、x,yだけcsvに出力するプログラム
#include <iostream>
#include <ros/ros.h>
#include <fstream>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>



std::ofstream ofs("2023-03-29_v1.0w0.525_amcl.csv");
// geometry_msgs::Quaternion robot_r;

// double robot_x = 0.0;
// double robot_y = 0.0;

double estimate_x = 0.0;
double estimate_y = 0.0;

void amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    estimate_x = msg->pose.pose.position.x;
    estimate_y = msg->pose.pose.position.y;    
}

// // odomの位置(ｘ,y,theta)をsub
// void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     robot_x = msg->pose.pose.position.x;
//     robot_y = msg->pose.pose.position.y;
//     robot_r = msg->pose.pose.orientation;

    
// }

// rolll, pitch, yawからクォータニオンにする関数

// メインの処理
int main(int argc, char **argv)
{

    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    // odomのsubscribe
    // ros::Subscriber odom_sub = nh.subscribe("/odom",10, odomCallback);
    // amcl_poseのsubscribe
    ros::Subscriber amcl_pose_sub = nh.subscribe("/amcl_pose", 10, amcl_poseCallback);
    ros::Rate rate(10.0);

    int cnt = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        // printf("robot_x, robot_y, robot_r: %f, %f, %f", robot_x, robot_y, tf::getYaw(robot_r));
        printf("cnt, estimate_x: %d,  %f\n", cnt ,estimate_x);
        // 実際の自己位置をcsvに出力
        ofs << estimate_x << ", " << estimate_y << std::endl;
        // if (cnt==100){
        //     break;
        // }
        cnt++;
    }

    rate.sleep();

}