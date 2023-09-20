#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // クォータニオンの取得
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    // クォータニオンをオイラー角に変換
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // オイラー角の出力
    ROS_INFO("Euler Angles: [Roll: %.2f, Pitch: %.2f, Yaw: %f]", roll, pitch, yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "quaternion_to_euler");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/beego/diff_drive_controller/odom", 1000, odomCallback);

    ros::spin();

    return 0;
}
