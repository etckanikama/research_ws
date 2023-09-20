#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

ros::Publisher pub;
tf2_ros::Buffer tfBuffer;



void rgb_to_hsv(float r, float g, float b, float &h, float &s, float &v) 
{

	double MAX = std::max((std::max(r,g)),b);
	double MIN = std::min((std::min(r,g)),b);
	v = MAX;

	if(MAX==MIN){
		h = 0;
		s = 0;
	}
	else{
		if(MAX == r) h = 60.0*(g-b)/(MAX-MIN) + 0;
		else if(MAX == g) h = 60.0*(b-r)/(MAX-MIN) + 120.0;
		else if(MAX == b) h = 60.0*(r-g)/(MAX-MIN) + 240.0;

		if(h > 360.0) h = h - 360.0;
		else if(h < 0) h = h + 360.0;
		s = (MAX-MIN)/MAX*100.0;
	}

	return;
}


void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    // Transform the point cloud from front_realsense_color_optical_frame to base_link
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", "front_realsense_color_optical_frame", ros::Time(0));
        tf2::doTransform(*input, transformed_cloud_msg, transformStamped);
        pcl::fromROSMsg(transformed_cloud_msg, cloud);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    for (const auto &point : cloud.points) {
        float h, s, v;
        rgb_to_hsv(point.r, point.g, point.b, h, s, v);
        
        if (h >= 0 && h <= 62 && s >= 0 && s <= 255 && v >= 235 && v <= 255) {
            cloud_filtered.points.push_back(point);

            std::cout << point.x << " " << point.y << " " << point.z << std::endl;
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header.frame_id = "base_link";
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_hsv_filter");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Subscriber sub = nh.subscribe("/front_realsense/depth/color/points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);

    ros::spin();

    return 0;
}
