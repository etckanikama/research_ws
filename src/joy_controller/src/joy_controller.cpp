#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <cmath>


double max_vel = 0.1;
// double max_omega = 0.525; //rad/s → 30 deg/s
double max_omega = 0.349; //rad/s ⇛20 deg/s
float x_joy = 0.0;
float w_joy = 0.0;

double beego_x   = 0.0;
double beego_y   = 0.0;
double beego_yaw = 0.0;

// 目標yaw角
double goal_yaw = 0.0; // 追加


sensor_msgs::Joy joy_data;
geometry_msgs::Twist cmd_vel;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    x_joy = joy_msg.axes[1];
    w_joy = joy_msg.axes[2];
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    beego_x = msg->pose.pose.position.x;
    beego_y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    beego_yaw = atan2(siny_cosp, cosy_cosp);
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "joy_controller");
//     ros::NodeHandle nh;
//     ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/beego/diff_drive_controller/cmd_vel", 10);
//     ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);
//     ros::Subscriber odom_sub = nh.subscribe("/beego/diff_drive_controller/odom", 1, odom_callback);

//     ros::Rate rate(10.0);
// //     while (ros::ok())
//     {
//         ros::spinOnce();

//         if (x_joy == 1) // ジョイスティックのX軸の入力が1の場合のみ
//         {
//             // 現在の角度と目標角度の差
//             double yaw_error = goal_yaw - beego_yaw;

//             // 角度の差が -π と π の間になるように調整
//             if (yaw_error > M_PI) {
//                 yaw_error -= 2 * M_PI;
//             } else if (yaw_error < -M_PI) {
//                 yaw_error += 2 * M_PI;
//             }

//             // 簡単な比例制御を使用して角速度を計算
//             double omega = yaw_error * 2.0; // ここで比例係数を調整可能

//             // 角速度を最大値で制限
//             omega = std::min(omega, max_omega);
//             omega = std::max(omega, -max_omega);

//             // 速度指令の更新
//             cmd_vel.linear.x = max_vel;
//             cmd_vel.angular.z = omega;
//         }
//         else 
//         {
//             // ジョイスティックの入力がない場合は停止
//             cmd_vel.linear.x = 0.0;
//             cmd_vel.angular.z = 0.0;
//         }

//         ROS_INFO("Odometry: x: [%f], y: [%f], yaw: [%f]", beego_x, beego_y, beego_yaw);

//         cmd_pub.publish(cmd_vel);
//         rate.sleep();
//     }
//     return 0;
// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/beego/diff_drive_controller/cmd_vel", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);
	ros::Subscriber odom_sub = nh.subscribe("/beego/diff_drive_controller/odom", 1, odom_callback);



    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        // printf("aaa\n");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        
        if (x_joy ==  1)
        {
            cmd_vel.linear.x = max_vel;
            // cmd_vel.angular.z = 0.0;
        }
    
        else 
        {
            cmd_vel.linear.x = 0.0;
            // cmd_vel.angular.z = 0.0;
        }

        if (w_joy == 1)
        {
            cmd_vel.angular.z = max_omega;
        }
        else if (w_joy == -1)
        {
            cmd_vel.angular.z = -max_omega;
        }
        else 
        {
            std::cout << "直進" << std::endl;
            cmd_vel.angular.z = 0.0;
        }

        // printf("x_joy:%f,  input_vel:%f\n", x_joy, cmd_vel.linear.x);
        // printf("w_joy:%f,  input_omega:%f\n", w_joy, cmd_vel.angular.z);
        ROS_INFO("linear.x: [%f], angular.z: [%f]", cmd_vel.linear.x, cmd_vel.angular.z);
        ROS_INFO("Odometry: x: [%f], y: [%f], yaw: [%f]", beego_x, beego_y, beego_yaw);


        cmd_pub.publish(cmd_vel);
        rate.sleep();
    }
    return 0;
}
