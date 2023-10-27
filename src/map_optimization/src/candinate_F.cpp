#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h> // PointCloudメッセージ型のためのヘッダ
#include <algorithm>
#include <cmath>
#include <vector>


const double DIVISION_INTERVAL = 0.1;

std::vector<geometry_msgs::Point> generate_divided_points(const geometry_msgs::PolygonStamped& polygon) {
    std::vector<geometry_msgs::Point> divided_points;

    double max_length = 0;
    int longest_side_index = -1;
    for (int i = 0; i < 4; ++i) {
        const auto& p1 = polygon.polygon.points[i];
        const auto& p2 = polygon.polygon.points[(i+1)%4];
        double length = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
        if (length > max_length) {
            max_length = length;
            longest_side_index = i;
        }
    }

    const auto& start = polygon.polygon.points[longest_side_index];
    const auto& end = polygon.polygon.points[(longest_side_index+1)%4];
    int divisions = static_cast<int>(max_length / DIVISION_INTERVAL);
    for (int i = 0; i <= divisions; ++i) {
        geometry_msgs::Point point;
        point.x = start.x + (end.x - start.x) * (i * DIVISION_INTERVAL / max_length);
        point.y = start.y + (end.y - start.y) * (i * DIVISION_INTERVAL / max_length);
        point.z = 0;
        divided_points.push_back(point);
    }

    return divided_points;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "candinate_white_line_node");
    ros::NodeHandle n;

    ros::Publisher poly_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon", 10);
    ros::Publisher poly_array_pub = n.advertise<jsk_recognition_msgs::PolygonArray>("polygon_array", 10);
    ros::Publisher divided_points_pub = n.advertise<sensor_msgs::PointCloud>("divided_points", 10); // sensor_msgs::PointCloudをpublishするためのPublisher
    ros::Publisher d_points_pub = n.advertise<sensor_msgs::PointCloud>("d_points", 10);

    // --------------dを作成------------------

    // x=0からx=10.7まで、10cm間隔で配列dを作成
    std::vector<geometry_msgs::Point32> d;
    for (double x = 0.0; x <= 10.7; x += 0.1) {
        geometry_msgs::Point32 point;
        point.x = x;
        point.y = 0.0;
        point.z = 0.0;
        d.push_back(point);
    }
    // x=10.5を固定して、yの負方向に-3まで10cm間隔で配列dに追加
    for (double y = -0.1; y >= -5.0; y -= 0.1) {
        geometry_msgs::Point32 point;
        point.x = 10.7;
        point.y = y;
        point.z = 0.0;
        d.push_back(point);
    }

    sensor_msgs::PointCloud d_cloud; 
    d_cloud.header.frame_id = "map";
    d_cloud.points = d;

    // ____________dを作成___________________

    // -----------Fを作成--------------------

    int polygon_num = 10;
    double rectangles[polygon_num][4][2] = {
        {{9.125, 0.545},{9.125, 0.47},{0.0, 0.47},{0.0, 0.545}},
        {{9.125, 1.935},{9.125, 0.545},{9.05, 0.545},{9.05, 1.935}},
        {{10.205, 1.935},{10.205, 1.86},{9.125, 1.86},{9.125, 1.935}},
        {{10.205,3.01},{10.205,1.935},{10.13,1.935},{10.13,3.01}},
        {{11.355,3.01},{11.355,1.76},{11.28,1.76},{11.28,3.01}},
        {{14.0,1.835},{14.0,1.76},{11.355,1.76},{11.355,1.835}},
        {{14.0,0.825},{14.0,0.75},{11.41,0.75},{11.41,0.825}},
        {{11.485,0.75},{11.485,-5.0},{11.41,-5.0},{11.41,0.75}},
        {{10.095,-0.545},{10.095,-5.0},{10.02,-5.0},{10.02,-0.545}},
        {{10.095, -0.47},{10.095,-0.545},{0,-0.545},{0,-0.47}}
    };

    jsk_recognition_msgs::PolygonArray polygons;
    for (int i = 0; i < polygon_num; ++i) {
        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id = "map";
        polygon.header.stamp = ros::Time::now();
        for (int j = 0; j < 4; ++j) {
            geometry_msgs::Point32 p;
            p.x = rectangles[i][j][0];
            p.y = rectangles[i][j][1];
            p.z = 0;
            polygon.polygon.points.push_back(p);
        }

        polygons.polygons.push_back(polygon);
    }
    // -----------Fを作成--------------------

    ros::Rate rate(10.0);
    while (ros::ok()) {
        std::cout << "白線マーカー表示中" << std::endl;
        ros::spinOnce();
        polygons.header.frame_id = "map";
        polygons.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud cloud; 
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time::now();
        d_cloud.header.stamp = ros::Time::now();

        int polygonIndex = 0; // 各ポリゴンを追跡するためのインデックス
        for (const auto& polygon : polygons.polygons) {
            auto divided_points = generate_divided_points(polygon);

            for (const auto& point : divided_points) {
                geometry_msgs::Point32 point32; 
                point32.x = point.x;
                point32.y = point.y;
                point32.z = point.z;

                cloud.points.push_back(point32);
                
                // 分割点の座標と、それがどのポリゴンに属するかをコンソールに出力
                std::cout << "Polygon Index: " << polygonIndex 
                        << ", Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            }
            
            polygonIndex++; // 次のポリゴンへ
        }

        poly_array_pub.publish(polygons);
        divided_points_pub.publish(cloud);
        d_points_pub.publish(d_cloud);
        rate.sleep();
    }

    return 0;
}
