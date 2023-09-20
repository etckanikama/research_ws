#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::PointCloud line_posi;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Quaternion robot_r;

std::ofstream ofs("白線のグローバル座標.csv");


// 初期速度指令値
double v = 0.0;
double omega = 0;
const double DT = 1.0 / 10.0;
double time_stamp = 0.0;
double robot_x = 0.0; //オドメトリ
double robot_y = 0.0; //オドメトリ

// roll, pitch, yawからクォータニオンにする関数
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

// クォータニオンをオイラーに変換
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}



// cmd_velとodomの値をcallback
void velCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    v = msg->twist.twist.linear.x;
    omega = msg->twist.twist.angular.z;
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;


}

// 白線点群を受け取る
void line_point_cb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    line_posi.points = msg->points;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_white_line_node");
    ros::NodeHandle nh;

    // Publisher
    
    
    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, velCallback);
    ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);


    ros::Rate rate(10.0);
    

    int cnt = 0;
    while (ros::ok())
    {   

        // 時間の加算
        ros::spinOnce();
        if (line_posi.points.size() > 0) 
        {
            // printf("白線点群のサイズ%ld \n",line_posi.points.size());
            // // printf("白線点群の座標%f \n",line_posi.points[0].x);
            // robot座標
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, robot_r); //yaw角に変換
            
            for (int j = 0; j < line_posi.points.size(); j++)//検出した白線の座標配列
            {
                    // オドメトリに準ずる白線の絶対座標に変換
                    double line_glx = line_posi.points[j].x * cosf(yaw) - line_posi.points[j].y * sinf(yaw) +  robot_x;
                    double line_gly = line_posi.points[j].x * sinf(yaw) + line_posi.points[j].y * cosf(yaw) +  robot_y;
                    // 時間と一緒にline_glx,line_glyをcsvに書き込む

                    ofs << cnt << ", " << line_glx << ", " <<line_gly << std::endl;
            }
        }
        cnt++;
        // if (cnt == 100)break;
        // // robot座標
        // double roll, pitch, yaw;
        // geometry_quat_to_rpy(roll, pitch, yaw, robot_r); //yaw角に変換
        // printf("robot_x:%f, robot_y:%f, yaw:%f \n",robot_x,robot_y,yaw);

        rate.sleep();

    }

}
