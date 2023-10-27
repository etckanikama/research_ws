#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

// dの座標を生成
std::vector<geometry_msgs::Point32> generate_d() {
    std::vector<geometry_msgs::Point32> d;

    for (double x = 0.0; x <= 10.7; x += 0.1) {
        geometry_msgs::Point32 point;
        point.x = x;
        point.y = 0.0;
        point.z = 0.0;
        d.push_back(point);
    }

    for (double y = -0.1; y >= -4.0; y -= 0.1) {
        geometry_msgs::Point32 point;
        point.x = 10.7;
        point.y = y;
        point.z = 0.0;
        d.push_back(point);
    }

    return d;
}

// Aの座標を設定
void set_A_coordinates(geometry_msgs::PolygonStamped& A, double x) {
    A.polygon.points.clear();

    if (x >= 10.6) {
        geometry_msgs::Point32 p1;
        p1.x = 0.8; p1.y = -0.5; p1.z = 0.0;
        A.polygon.points.push_back(p1);

        geometry_msgs::Point32 p2;
        p2.x = 0.8; p2.y = -2.5; p2.z = 0.0;
        A.polygon.points.push_back(p2);

        geometry_msgs::Point32 p3;
        p3.x = -0.8; p3.y = -2.5; p3.z = 0.0;
        A.polygon.points.push_back(p3);

        geometry_msgs::Point32 p4;
        p4.x = -0.8; p4.y = -0.5; p4.z = 0.0;
        A.polygon.points.push_back(p4);
    } else {
        // 通常のAの座標
        geometry_msgs::Point32 p1;
        p1.x = 0.5; p1.y = 0.8; p1.z = 0.0;
        A.polygon.points.push_back(p1);

        geometry_msgs::Point32 p2;
        p2.x = 2.5; p2.y = 0.8; p2.z = 0.0;
        A.polygon.points.push_back(p2);

        geometry_msgs::Point32 p3;
        p3.x = 2.5; p3.y = -0.8; p3.z = 0.0;
        A.polygon.points.push_back(p3);

        geometry_msgs::Point32 p4;
        p4.x = 0.5; p4.y = -0.8; p4.z = 0.0;
        A.polygon.points.push_back(p4);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_translated_polygons");
    ros::NodeHandle n;

    ros::Publisher A_pub = n.advertise<geometry_msgs::PolygonStamped>("moved_A", 10); // AをpublishするためのPublisher
    ros::Publisher d_pub = n.advertise<geometry_msgs::PointStamped>("d_coordinates", 10);

    ros::Rate rate(10.0);

    std::vector<geometry_msgs::Point32> d = generate_d();
    int d_index = 0; // dの座標を追跡するためのインデックス

    geometry_msgs::PolygonStamped A;
    A.header.frame_id = "map";

    while (ros::ok()) {
        geometry_msgs::PolygonStamped translated_A;

        translated_A.header.frame_id = "map";
        translated_A.header.stamp = ros::Time::now();

        // dの座標を取得
        geometry_msgs::Point32 d_point = d[d_index];

        // Aの座標を設定
        set_A_coordinates(A, d_point.x);

        // Aをdの座標に移動
        for (const auto& point : A.polygon.points) {
            geometry_msgs::Point32 translated_point;
            translated_point.x = point.x + d_point.x;
            translated_point.y = point.y + d_point.y;
            translated_point.z = point.z;
            translated_A.polygon.points.push_back(translated_point);
        }

        // translated_Aとdの座標をpublish
        A_pub.publish(translated_A);

        geometry_msgs::PointStamped d_stamped;
        d_stamped.header.stamp = ros::Time::now();
        d_stamped.header.frame_id = "map";
        d_stamped.point.x = d_point.x;
        d_stamped.point.y = d_point.y;
        d_stamped.point.z = d_point.z;
        d_pub.publish(d_stamped);

        d_index++;
        if (d_index >= d.size()) {
            d_index = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
