#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

double max_vel = 0.5; //0.5 m/s 
// double max_omega = 0.2;
// double max_omega = 0.525; //rad/s → 30 deg/s
double max_omega = 1.0472; //rad/s → 60 deg/s 


float x_joy = 0.0;
float w_joy = 0.0;
float stop_joy[3] = {0.0};




sensor_msgs::Joy joy_data;
geometry_msgs::Twist cmd_vel;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    x_joy = joy_msg.axes[1];
    w_joy = joy_msg.axes[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // シミュレーション
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/beego/diff_drive_controller/cmd_vel", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);



    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        printf("aaa\n");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        
        if (x_joy ==  1)
        {
            cmd_vel.linear.x = max_vel;
        }
    
        else 
        {
            cmd_vel.linear.x = 0.0;
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
            cmd_vel.angular.z = 0.0;
        }

        printf("x_joy:%f,  input_vel:%f\n", x_joy, cmd_vel.linear.x);
        printf("w_joy:%f,  input_omega:%f\n", w_joy, cmd_vel.angular.z);

        cmd_pub.publish(cmd_vel);
        rate.sleep();
    }
    return 0;
}
