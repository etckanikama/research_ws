#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

nav_msgs::Odometry pos;
geometry_msgs::Twist pub_msg;


//　指定した座標付近で停止
int near_position(double yaw, double goal_yaw)
{
	return (-0.05 < yaw - goal_yaw && yaw - goal_yaw < 0.05);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	pos.header = msg->header;
	pos.child_frame_id = msg->child_frame_id;
	pos.pose = msg->pose;
	pos.twist = msg->twist;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "intro_node");

	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate loop_rate(10);

	ros::Time start_time=ros::Time::now();
	while (ros::ok() && ros::Time::now()- start_time < ros::Duration(40))
	{
		ros::spinOnce();
		pub_msg.angular.z = 1.0;
		pub.publish(pub_msg);
		loop_rate.sleep();
	}
	while (ros::ok())
	{
		ros::spinOnce();

		tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
		tf::Matrix3x3 m(quat);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		std::cout << "yaw;" << yaw << std::endl;
		
		if (near_position(yaw, 0.0))
        {
            pub_msg.angular.z = 0.0;
        }
		else{
			pub_msg.angular.z = 0.3;
		}
		pub.publish(pub_msg);

		loop_rate.sleep();
	}

	return 0;
}
