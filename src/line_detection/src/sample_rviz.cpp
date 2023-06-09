#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_left_pub = n.advertise<visualization_msgs::Marker>("visualization_left_marker", 10);
  ros::Publisher marker_right_pub = n.advertise<visualization_msgs::Marker>("visualization_right_marker", 10);

  ros::Rate r(30);


  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points_right, points_left, line_strip, line_list;
    points_right.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "front_realsense_link";
    points_right.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points_right.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points_right.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points_right.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points_left.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "front_realsense_link";
    points_left.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points_left.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points_left.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points_left.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points_right.id = 0;
    points_left.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points_right.type = visualization_msgs::Marker::POINTS;
    points_left.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points_right.scale.x = 0.05;
    points_right.scale.y = 0.05;
    points_left.scale.x = 0.05;
    points_left.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
    points_right.color.g = 1.0f;
    points_right.color.a = 1.0;
    points_left.color.g = 1.0f;
    points_left.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(HELIX)%
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 50; ++i)
    {
    //   float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    //   float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
      // float x = 2.5;
      float y = 0.5;
      float z = 0;
      
      geometry_msgs::Point p;
      

      if (i > 30)
      {
        p.x = 3.0;
        p.y = y - (i-30)*0.1;
        p.z = z;
        // std::cout << p.x << std::endl;
        // std::cout << p.y << std::endl;
      }
      else
      {
        p.x = (int32_t)i*0.1;
        p.y = y;
        p.z = z;
      }
      std::cout << i << " " << p.y << std::endl;
      
      points_right.points.push_back(p);
    //   line_strip.points.push_back(p);

    //   // The line list needs two points for each line
    //   line_list.points.push_back(p);
    //   p.z += 1.0;
    //   line_list.points.push_back(p);
    }

    for (uint32_t i = 0; i < 40; ++i)
    {
    //   float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    //   float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
      float y = -0.5;
      float z = 0;

      geometry_msgs::Point p;
      

      if (i > 20)
      {
        p.x = 2.0;
        p.y = y - (i-20)*0.1;
        p.z = z;
        // std::cout << p.x << std::endl;
        // std::cout << p.y << std::endl;
      }
      else
      {
        p.x = (int32_t)i*0.1;
        p.y = y;
        p.z = z;
      }

      points_left.points.push_back(p);
    //   line_strip.points.push_back(p);

    //   // The line list needs two points for each line
    //   line_list.points.push_back(p);
    //   p.z += 1.0;
    //   line_list.points.push_back(p);
    }

// %EndTag(HELIX)%
    // for (int i = 0; i < 100; i++)
    // {
    //     std::cout << points.points[i].x << std::endl;
    // }
    

    marker_left_pub.publish(points_right);
    marker_right_pub.publish(points_left);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);

    r.sleep();


  }
}