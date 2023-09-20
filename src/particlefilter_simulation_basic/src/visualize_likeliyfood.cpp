#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

// Global variables to store the latest poses and weights
geometry_msgs::PoseArray latest_poses;
std_msgs::Float32MultiArray latest_weights;
bool got_poses = false;
bool got_weights = false;

void posesCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    latest_poses = *msg;
    got_poses = true;
}

void weightsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    latest_weights = *msg;
    got_weights = true;

    // printf("Received weights: [");
    // for (size_t i = 0; i < latest_weights.data.size(); i++) {
    //     printf("%f", latest_weights.data[i]);
    //     if (i != latest_weights.data.size() - 1) {
    //         printf(", ");
    //     }
    // }
    // printf("]\n");

    // for (float weight : latest_weights.data) {
    //     if (weight > 0.0021) {
    //         ROS_WARN("Received weight value greater than 0.002: %f", weight);
    //     }
    // }
}

void hsv_to_rgb(float h, float s, float v, float &r, float &g, float &b)
{

    h = h * 3.0f; // Convert to [0, 6)
    int i = static_cast<int>(floor(h));
    float f = h - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (i)
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    default: // case 5:
        r = v;
        g = p;
        b = q;
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_particles");
    ros::NodeHandle nh;

    ros::Subscriber poses_sub = nh.subscribe("/particle_cloud", 10, posesCallback);
    ros::Subscriber weights_sub = nh.subscribe("/converted_weights", 10, weightsCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/particle_markers", 10);

    ros::Rate loop_rate(10);
    float rt,gt,bt;
    hsv_to_rgb((0), 1.0, 1.0, rt, gt, bt);
    std::cout << " 0の時 "<< rt << " " << gt << " " << bt<< std::endl;
    hsv_to_rgb((0.5), 1.0, 1.0, rt, gt, bt);
    std::cout << " 0.5の時 "<< rt << " " << gt << " " << bt<< std::endl;
    hsv_to_rgb((1), 1.0, 1.0, rt, gt, bt);
    std::cout <<" 1の時 "<< rt << " " << gt << " " << bt<< std::endl;
    while (ros::ok())
    {

        if (got_poses && got_weights)
        {
            visualization_msgs::MarkerArray marker_array;

            for (int i = 0; i < latest_poses.poses.size(); i++)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = latest_poses.header.frame_id;
                marker.header.stamp = ros::Time::now();
                marker.ns = "particles";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = latest_poses.poses[i];
                marker.scale.x = 0.03;
                marker.scale.y = 0.03;
                marker.scale.z = 0.03;
                marker.color.a = 1.0;
                float r, g, b;
                
                hsv_to_rgb((latest_weights.data[i] * 500.0f), 1.0, 1.0, r, g, b);
                marker.color.r = r;
                marker.color.g = g;
                marker.color.b = b;


                marker_array.markers.push_back(marker);
            }

            marker_pub.publish(marker_array);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
