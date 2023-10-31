#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

nav_msgs::Odometry pos;
geometry_msgs::Twist pub_msg;


//　指定した座標付近で停止
int near_position(double goal_x, double goal_y)
{
	double difx = pos.pose.pose.position.x - goal_x;
	double dify = pos.pose.pose.position.y - goal_y;
	return (sqrt(difx * difx + dify * dify) < 0.1);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	pos.header = msg->header;
	pos.child_frame_id = msg->child_frame_id;
	pos.pose = msg->pose;
	pos.twist = msg->twist;
}

//　直線追従を行う関数
void line_GL(double x, double y, double th)
{
	// 制御のパラメータ(調整必須)
	const double k_eta = 60;
	const double k_phai = 30;
	const double k_w = 40;

	// 速度と角速度の最大値
	const double v_max = 0.3;
	const double w_max = 0.2;

	tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
	tf::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	// 現在のロボットの位置、姿勢
	double x0 = pos.pose.pose.position.x;
	double y0 = pos.pose.pose.position.y;
	double theta = yaw;

	// 速度
	double v0 = 0.3;

	// 現在の角速度
	double w0 = pos.twist.twist.angular.z;

	// ロボットと直線の距離
	double eta = 0;
	if (th == M_PI / 2.0)
		eta = -(x0 - x);
	else if (th == -M_PI / 2.0)
		eta = x0 - x;
	else if (abs(th) < M_PI / 2.0)
		eta = (-tan(th) * x0 + y0 - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
	else
		eta = -(-tan(th) * x0 + y0 - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
	if (eta > 4.0)
		eta = 4.0;
	else if (eta < -4.0)
		eta = -4.0;

	// 直線に対するロボットの向き
	double phai = theta - th;
	while (phai <= -M_PI || M_PI <= phai)
	{
		if (phai <= -M_PI)
			phai = phai + 2 * M_PI;
		else
			phai = phai - 2 * M_PI;
	}

	// 目標となるロボットの角速度と現在の角速度の差
	double w_diff = w0;

	// 角速度
	double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w_diff) * 0.01;
	if (w > w_max)
		w = w_max;
	else if (w < -w_max)
		w = -w_max;

	// 並進速度
	double v = v0;
	if (v > v_max)
		v = v_max;
	else if (v < -v_max)
		v = 0.0;
		
	// std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	// std::cout << "v: " << v << "   w: " << w << std::endl;
	// std::cout << "(x,y,theta) = (" << x0 << "," << y0 << "," << theta << ")" << std::endl;
	// std::cout << "------------------------------" << std::endl;

	// 送信する値
	pub_msg.linear.x = v;
	pub_msg.linear.y = 0.0;
	pub_msg.linear.z = 0.0;
	pub_msg.angular.x = 0.0;
	pub_msg.angular.y = 0.0;
	pub_msg.angular.z = w;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "intro_node");

	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	pos.pose.pose.position.x = 0.0;
	pos.pose.pose.position.y = 0.0;

	// 直線を指定するための値
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();

		// x = 0.0;
		// y = 0.5;
		// theta = 0.0;
		// x = 0.0;
		// y = 1.0;
		// theta = M_PI / 4.0;
		x = 10.0;
		y = 0.0;
		theta = 0.0;
		line_GL(x, y, theta);
		if (near_position(10.0, 0.0))
            {
                pub_msg.linear.x = 0.0;
                pub_msg.angular.z = 0.0;
				std::cout << "pos.pose.pose.position.x;" << pos.pose.pose.position.x << std::endl;
				std::cout << "pos.pose.pose.position.y;" << pos.pose.pose.position.y << std::endl;
            }
		pub.publish(pub_msg);

		loop_rate.sleep();
	}

	return 0;
}
