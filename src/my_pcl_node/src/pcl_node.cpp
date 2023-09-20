#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub, pub_inliers, pub_marker;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_inliers);

    sensor_msgs::PointCloud2::Ptr cloud_inliers2(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_inliers, *cloud_inliers2);
    pub_inliers.publish(cloud_inliers2);

    // Calculate centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_inliers, centroid);

    // Prepare marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_msg->header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set marker points (start and end)
    geometry_msgs::Point start, end;
    start.x = centroid[0];
    start.y = centroid[1];
    start.z = centroid[2];

    // Set the end point to be the vector direction from the centroid
    end.x = start.x + coefficients->values[0];
    end.y = start.y + coefficients->values[1];
    end.z = start.z + coefficients->values[2];

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.1;
    marker.scale.y = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Publish the marker
    pub_marker.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/front_realsense/depth/color/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_normal", 1);
    pub_inliers = nh.advertise<sensor_msgs::PointCloud2>("/plane_inliers", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::spin();
}
