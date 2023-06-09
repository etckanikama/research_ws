#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "white_lane_node");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher m1_pub = nh.advertise<visualization_msgs::Marker>("m1", 1);
  ros::Publisher m2_pub = nh.advertise<visualization_msgs::Marker>("m2", 1);
  ros::Publisher m3_pub = nh.advertise<visualization_msgs::Marker>("m3", 1);
  ros::Publisher m4_pub = nh.advertise<visualization_msgs::Marker>("m4", 1);
  ros::Publisher m5_pub = nh.advertise<visualization_msgs::Marker>("m5", 1);
  ros::Publisher m6_pub = nh.advertise<visualization_msgs::Marker>("m6", 1);
  ros::Publisher m7_pub = nh.advertise<visualization_msgs::Marker>("m7", 1);
  ros::Publisher m8_pub = nh.advertise<visualization_msgs::Marker>("m8", 1);

  ros::Publisher robot_posi_pub = nh.advertise<visualization_msgs::Marker>("robot_position", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    
    visualization_msgs::Marker m1;
    // m1.header.frame_id = "base_link";
    m1.header.frame_id = "base_link";
    m1.header.stamp = ros::Time::now();
    m1.ns = "basic_shapes";
    m1.id = 0;

    m1.type = visualization_msgs::Marker::CUBE;
    m1.action = visualization_msgs::Marker::ADD;
    m1.lifetime = ros::Duration();

    m1.scale.x = 5.47;
    m1.scale.y = 0.075;
    m1.scale.z = 0.01;
    m1.pose.position.x = 2.735;
    m1.pose.position.y = -0.5;
    m1.pose.position.z = 0;
    m1.pose.orientation.x = 0;
    m1.pose.orientation.y = 0;
    m1.pose.orientation.z = 0;
    m1.pose.orientation.w = 1;
    m1.color.r = 0.0f;
    m1.color.g = 1.0f;
    m1.color.b = 0.0f;
    m1.color.a = 1.0f;
    
    visualization_msgs::Marker m2;
    m2.header.frame_id = "base_link";
    m2.header.stamp = ros::Time::now();
    m2.ns = "basic_shapes";
    m2.id = 0;

    m2.type = visualization_msgs::Marker::CUBE;
    m2.action = visualization_msgs::Marker::ADD;
    m2.lifetime = ros::Duration();

    m2.scale.x = 5.47;
    m2.scale.y = 0.075;
    m2.scale.z = 0.01;
    m2.pose.position.x = 2.735;
    m2.pose.position.y = 0.5;
    m2.pose.position.z = 0;
    m2.pose.orientation.x = 0;
    m2.pose.orientation.y = 0;
    m2.pose.orientation.z = 0;
    m2.pose.orientation.w = 1;
    m2.color.r = 0.0f;
    m2.color.g = 1.0f;
    m2.color.b = 0.0f;
    m2.color.a = 1.0f;


    visualization_msgs::Marker m3;
    m3.header.frame_id = "base_link";
    m3.header.stamp = ros::Time::now();
    m3.ns = "basic_shapes";
    m3.id = 0;

    m3.type = visualization_msgs::Marker::CUBE;
    m3.action = visualization_msgs::Marker::ADD;
    m3.lifetime = ros::Duration();

    m3.scale.x = 0.075;
    m3.scale.y = 2;
    m3.scale.z = 0.01;
    m3.pose.position.x = 5.47;
    m3.pose.position.y = 1.5;
    m3.pose.position.z = 0;
    m3.pose.orientation.x = 0;
    m3.pose.orientation.y = 0;
    m3.pose.orientation.z = 0;
    m3.pose.orientation.w = 1;
    m3.color.r = 0.0f;
    m3.color.g = 1.0f;
    m3.color.b = 0.0f;
    m3.color.a = 1.0f;

    visualization_msgs::Marker m4;
    m4.header.frame_id = "base_link";
    m4.header.stamp = ros::Time::now();
    m4.ns = "basic_shapes";
    m4.id = 0;

    m4.type = visualization_msgs::Marker::CUBE;
    m4.action = visualization_msgs::Marker::ADD;
    m4.lifetime = ros::Duration();

    m4.scale.x = 0.075;
    m4.scale.y = 4;
    m4.scale.z = 0.01;
    m4.pose.position.x = 5.47;
    m4.pose.position.y = -2.5;
    m4.pose.position.z = 0;
    m4.pose.orientation.x = 0;
    m4.pose.orientation.y = 0;
    m4.pose.orientation.z = 0;
    m4.pose.orientation.w = 1;
    m4.color.r = 0.0f;
    m4.color.g = 1.0f;
    m4.color.b = 0.0f;
    m4.color.a = 1.0f;

    visualization_msgs::Marker m5;
    m5.header.frame_id = "base_link";
    m5.header.stamp = ros::Time::now();
    m5.ns = "basic_shapes";
    m5.id = 0;

    m5.type = visualization_msgs::Marker::CUBE;
    m5.action = visualization_msgs::Marker::ADD;
    m5.lifetime = ros::Duration();

    m5.scale.x = 0.075;
    m5.scale.y = 2;
    m5.scale.z = 0.01;
    m5.pose.position.x = 6.47;
    m5.pose.position.y = 1.5;
    m5.pose.position.z = 0;
    m5.pose.orientation.x = 0;
    m5.pose.orientation.y = 0;
    m5.pose.orientation.z = 0;
    m5.pose.orientation.w = 1;
    m5.color.r = 0.0f;
    m5.color.g = 1.0f;
    m5.color.b = 0.0f;
    m5.color.a = 1.0f;

    visualization_msgs::Marker m6;
    m6.header.frame_id = "base_link";
    m6.header.stamp = ros::Time::now();
    m6.ns = "basic_shapes";
    m6.id = 0;

    m6.type = visualization_msgs::Marker::CUBE;
    m6.action = visualization_msgs::Marker::ADD;
    m6.lifetime = ros::Duration();

    m6.scale.x = 0.075;
    m6.scale.y = 4;
    m6.scale.z = 0.01;
    m6.pose.position.x = 6.47;
    m6.pose.position.y = -2.5;
    m6.pose.position.z = 0;
    m6.pose.orientation.x = 0;
    m6.pose.orientation.y = 0;
    m6.pose.orientation.z = 0;
    m6.pose.orientation.w = 1;
    m6.color.r = 0.0f;
    m6.color.g = 1.0f;
    m6.color.b = 0.0f;
    m6.color.a = 1.0f;

    visualization_msgs::Marker m7;
    m7.header.frame_id = "base_link";
    m7.header.stamp = ros::Time::now();
    m7.ns = "basic_shapes";
    m7.id = 0;

    m7.type = visualization_msgs::Marker::CUBE;
    m7.action = visualization_msgs::Marker::ADD;
    m7.lifetime = ros::Duration();

    m7.scale.x = 7.0;
    m7.scale.y = 0.075;
    m7.scale.z = 0.01;
    m7.pose.position.x = 10.0; // 中心7で上下に3mずつ
    m7.pose.position.y = 0.5;
    m7.pose.position.z = 0;
    m7.pose.orientation.x = 0;
    m7.pose.orientation.y = 0;
    m7.pose.orientation.z = 0;
    m7.pose.orientation.w = 1;
    m7.color.r = 0.0f;
    m7.color.g = 1.0f;
    m7.color.b = 0.0f;
    m7.color.a = 1.0f;

    visualization_msgs::Marker m8;
    m8.header.frame_id = "base_link";
    m8.header.stamp = ros::Time::now();
    m8.ns = "basic_shapes";
    m8.id = 0;

    m8.type = visualization_msgs::Marker::CUBE;
    m8.action = visualization_msgs::Marker::ADD;
    m8.lifetime = ros::Duration();

    m8.scale.x = 7.0;
    m8.scale.y = 0.075;
    m8.scale.z = 0.01;
    m8.pose.position.x = 10.0; // 中心7で上下に3mずつ
    m8.pose.position.y = -0.5;
    m8.pose.position.z = 0;
    m8.pose.orientation.x = 0;
    m8.pose.orientation.y = 0;
    m8.pose.orientation.z = 0;
    m8.pose.orientation.w = 1;
    m8.color.r = 0.0f;
    m8.color.g = 1.0f;
    m8.color.b = 0.0f;
    m8.color.a = 1.0f;
    // visualization_msgs::Marker robot_maker;
    // robot_maker.header.frame_id = "base_link";
    // robot_maker.header.stamp = ros::Time::now();
    // robot_maker.ns = "basic_shapes";
    // robot_maker.id = 0;

    // robot_maker.type = visualization_msgs::Marker::CUBE;
    // robot_maker.action = visualization_msgs::Marker::ADD;
    // robot_maker.lifetime = ros::Duration();

    // robot_maker.scale.x = 0.2;
    // robot_maker.scale.y = 0.2;
    // robot_maker.scale.z = 0.2;
    // robot_maker.pose.position.x = 0;
    // robot_maker.pose.position.y = 0;
    // robot_maker.pose.position.z = 0;
    // robot_maker.pose.orientation.x = 0;
    // robot_maker.pose.orientation.y = 0;
    // robot_maker.pose.orientation.z = 0;
    // robot_maker.pose.orientation.w = 1;
    // robot_maker.color.r = 0.0f;
    // robot_maker.color.g = 1.0f;
    // robot_maker.color.b = 0.0f;
    // robot_maker.color.a = 1.0f;

    m1_pub.publish(m1);
    m2_pub.publish(m2);
    m3_pub.publish(m3);
    m4_pub.publish(m4);
    m5_pub.publish(m5);
    m6_pub.publish(m6);
    m7_pub.publish(m7);
    m8_pub.publish(m8);

    // robot_posi_pub.publish(robot_maker);
  


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}