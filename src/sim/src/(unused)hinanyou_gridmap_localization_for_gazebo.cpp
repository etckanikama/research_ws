#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gazebo_msgs/ModelStates.h>
#include <limits>
#include <cstdlib>  // getenvを使うために必要
#include <sstream> // std::ostringstreamを使用するために必要
#include <cmath> // std::fmod関数のために必要


#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;

// イチピクセルの範囲を表す構造体：グリッドマップ
struct CornerPoints {
    double A1_x, A1_y; // 左上
    double A2_x, A2_y; // 右上
    double A3_x, A3_y; // 右下
    double A4_x, A4_y; // 左下
    int8_t value;      // 0 or 100(白線？)
};



std::vector<CornerPoints> corner_points;
pcl::PointCloud<pcl::PointXYZ> downsampled_cloud; //センサー情報を受け取る
bool map_processed = false; // 事前計算の完了状態を追跡


// 座標変換とフィルタリングを行う関数
std::vector<CornerPoints> extractPointsWithinTransformedRange(const std::vector<CornerPoints>& allPoints, double originX, double originY, double originYaw, double forwardRange, double sideRange) {
    std::vector<CornerPoints> filteredPoints;

    for (const auto& point : allPoints) {
        // init_yawを考慮した座標変換
        double dx = point.A1_x - originX;
        double dy = point.A1_y - originY;

        double transformedX = dx * cos(-originYaw) - dy * sin(-originYaw);
        double transformedY = dx * sin(-originYaw) + dy * cos(-originYaw);

        // 変換された座標系での範囲チェック
        if (transformedX >= 0 && transformedX <= forwardRange && abs(transformedY) <= sideRange) {
            filteredPoints.push_back(point);
        }
    }

    return filteredPoints;
}


// map情報コールバック
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (map_processed) return; // 地図情報は一度だけ処理する
    double resolution = msg->info.resolution;
    auto origin = msg->info.origin.position;

    int width = msg->info.width;
    int height = msg->info.height;

    corner_points.clear(); // 以前のデータをクリア
    cout << "一回しか呼ばれないはず" << endl;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width; // 1Dインデックスint index = x + y * width; // 1Dインデックス
            double map_x = x * resolution + origin.x;
            double map_y = y * resolution + origin.y;

            CornerPoints points;
            points.A1_x = map_x;
            points.A1_y = map_y + resolution;
            points.A2_x = map_x + resolution;
            points.A2_y = map_y + resolution;
            points.A3_x = map_x + resolution;
            points.A3_y = map_y;
            points.A4_x = map_x;
            points.A4_y = map_y;
            points.value= msg->data[index];

            corner_points.push_back(points);
        }
    }
    cout << static_cast<int>(msg->data[0]) << endl; // 修正


    map_processed = true; // 事前計算が完了したことを示す
    ROS_INFO("Map processing complete.");
}





int main(int argc, char **argv) {
    ros::init(argc, argv, "map_corners_listener");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);

    double init_x=0.0,init_y=0.0,init_yaw=0.0;//あとで入力変えられるようにする


    ros::Rate rate(10.0);


    while (ros::ok()) 
    {

        ros::spinOnce(); // コールバック関数を実行
        // cout << "真値 " << beego_x << " " << beego_y << " " << beego_yaw<< endl; 
        cout << "マッププロセス " << map_processed << endl;
        if (map_processed) {
            
            // init_x, init_yからxの正方向に3m, yの正負2mの領域の点を抽出
            std::vector<CornerPoints> filteredPoints = extractPointsWithinTransformedRange(corner_points, init_x, init_y, init_yaw, 3.0, 2.0);

            // フィルタリングされた点に基づいて何らかの処理を行う（例: 表示）
            for (const auto& point : filteredPoints) {
                cout << "Filtered Point_A1: " << point.A1_x << ", " << point.A1_y << endl;
                // cout << "Filtered Point_A2: " << point.A2_x << ", " << point.A2_y << endl;
            }

            // // init_x, init_yの更新処理（仮に値を更新する例）
            // // ここで新しいinit_x, init_yの値に基づいて次のループでフィルタリングが行われる
            // init_x += 0.5; // 例として0.5メートルずつ移動すると仮定
            // init_y += 0.0; // Y方向の移動はなしとする
        }





        rate.sleep(); // 次のループまで待機
    }
    return 0;
}
