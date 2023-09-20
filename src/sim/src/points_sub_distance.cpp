#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

double distance(double xA, double yA, double xB, double yB, double xP, double yP) {
    double APx = xP - xA;
    double APy = yP - yA;
    double ABx = xB - xA;
    double ABy = yB - yA;

    double cross_product = APx * ABy - APy * ABx;
    double length_AB = std::sqrt(ABx * ABx + ABy * ABy);
    double distance = std::abs(cross_product) / length_AB;

    return distance;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    double WHITE_CENTERLINE_COODINATE[2][4] = {{0.0, 0.5, 9.0, 0.5}, {0.0, -0.5, 4.0, -0.5}};

    for (const auto& point : cloud.points) {
        for (int i = 0; i < 2; ++i) {
            double xA = WHITE_CENTERLINE_COODINATE[i][2];
            double yA = WHITE_CENTERLINE_COODINATE[i][3];
            double xB = WHITE_CENTERLINE_COODINATE[i][0];
            double yB = WHITE_CENTERLINE_COODINATE[i][1];

            double result = distance(xA, yA, xB, yB, point.x, point.y);

            ROS_INFO("点 (%f, %f) と線分 %c%c との距離は %f です。", point.x, point.y, (char)('A' + i * 2), (char)('B' + i * 2), result);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_calculator");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_cloud_topic", 1, pointCloudCallback);

    ros::spin();

    return 0;
}
