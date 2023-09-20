#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Joy.h>
// #include <sensor_msgs/point_cloud_conversion.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <time.h>
#include <chrono>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <assert.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SIZE_OF_ARRAY(array) (sizeof(array) / sizeof(array[0]))
using namespace std;


/*
    実験したい任意の座標（グローバル）における尤度分布の平均と標準偏差を求める
    　ユーザー
    　　・実験したい任意の座標を指定：global_initial_x, global_y
    　　・指定した位置にロボットを実際に置くor rosbagをとる：姿勢は0.0にする(lidarで揃える)
    　　・角度に関しては、初期姿勢:global_initial_yaw
*/


sensor_msgs::PointCloud line_posi;
sensor_msgs::PointCloud2 down_sample_line_posi;
geometry_msgs::PoseArray particle_cloud; // poseArrayの値の入れ方：https://answers.ros.org/question/251586/rostopic-pub-geometry_msgsposearray-example/
pcl::PointCloud<pcl::PointXYZ> downsampled_cloud, line_cloud;



// パーティクル数
const int PARTICLE_NUM = 400;
const double dt = 1.0 / 10.0;                 // 周期
double time_stamp = 0.0;                      // 時刻




const int POLYGON_NUM = 10; // ポリゴンの数l棟は8,sbは10
double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 13.5, 1.76, 1.835}, {11.41, 13.5, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.6}, {10.02, 10.095, -0.545, -3.6}, {0, 10.095, -0.47, -0.545}};
// const int POLYGON_NUM = 25; // sbのポリゴンの数は駐車場加えて24(x_min,x_max,y_min,y_max)
// double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 12.6, 1.76, 1.835}, {11.41, 12.6, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.6}, {10.02, 10.095, -0.545, -3.6}, {0, 10.095, -0.47, -0.545},
//                                             {0.92, 0.95, -1.465, -3.61},{0.95, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
//                                             {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61}};
// const int POLYGON_NUM = 3; 
// double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{11.41, 11.485, 0.75, -5.0}, {10.02, 10.095, -0.545, -5.0},{0, 10.095, -0.47, -0.545}};

// 白線ポリゴンの中心座標：{A1(X1,Y1),B1(X2,Y2)},{A2(X1,Y1),B2(X2,Y2)}...{A10(X1,Y1),B10(X2,Y2)}
// double WHITE_CENTERLINE_COODINATE[POLYGON_NUM][4] = {{0.0, 0.5075, 9.0875, 0.5075}, {9.0875, 0.5075, 9.0875, 1.8975}, {9.0875, 1.8975, 10.1675, 1.8975}, {10.1675, 1.8975, 10.1675, 3.01}, {11.3175, 3.01, 11.3175, 1.7975}, {11.3175, 1.7975, 12.63, 1.7975}, {11.4475, 0.7875, 13.5, 0.7875}, {11.4475, 0.7875, 11.4475, -5.0}, {10.0575, -0.5075, 11.4475, -5.0}, {0, -0.5075, 10.0575, -0.5075}};

double avg = 0.0, sig = 0.01; // 平均0,標準偏差0.01(初期の散らばり具合)
// std::vector<double> particle_value(particle_num); // particleの重み
double particle_value[PARTICLE_NUM] = {0.0};
int flg = 0;



// 乱数生成エンジン
std::random_device seed;
std::mt19937 engine(seed()); // メルセンヌ・ツイスタ



// callback
void line_point_cb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    if (flg == 0)
    {
        line_posi.points = msg->points;
        flg = 1;
    }

    // std::cout <<"pointの数" << line_posi.points.size() << std::endl;
}

// ボクセルダウンサンプリングをしたあとの点群を読み込む
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // PointCloud2メッセージをPCLのPointCloud型に変換する
    if (flg == 0)
    {
        pcl::fromROSMsg(*msg, downsampled_cloud);
        flg = 1;
    }
}

void lineCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // PointCloud2メッセージをPCLのPointCloud型に変換する
    //   if (flg==0)
    //   {
    pcl::fromROSMsg(*msg, line_cloud);
    // cout << line_cloud.points.size() << endl;
    //     flg=1;
    //   }
}

// オイラーをクォータニオンに変換
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

// クォータニオンをオイラーに変換
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // rpy are Pass by Reference
}

// yaw角をtfのクォータニオン型に変換する関数
tf::Quaternion getYawQuaternion(double yaw)
{
    tf::Quaternion q;
    q.setRPY(0, 0, yaw); // RPY stands for Roll, Pitch, Yaw
    return q;
}

// 中心座標とサイズを指定して二次元座標配列を生成する関数
vector<vector<pair<double, double>>> generateCoordinateArray(pair<double, double> center, int size, double width)
{
    vector<vector<pair<double, double>>> coordinates;
    double step = 2.0 * width / (size - 1); // ステップの計算
    double startX = center.first + width;
    double startY = center.second + width;
    // std::cout << startX << " , " << startY << std::endl;

    coordinates.resize(size, vector<pair<double, double>>(size));
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            double x = startX - i * step;
            double y = startY - j * step;
            coordinates[i][j] = make_pair(x, y);
        }
    }

    return coordinates;
}

// // 内積を求める関数ベクトルAとベクトルBの内積
// double dot(double Ax, double Ay, double Bx, double By) {
//     // cout << Ax << " " <<  Bx << " " << Ax*Bx << " " << Ay << " " << " " << By << " " <<
//     return Ax*Bx + Ay*By;
// }

// // 外積を求める関数
// double cross_product(double Ax, double Ay, double Bx, double By) {
//     return Ax*By - Bx*Ay;
// }

// // 点A(x1,y1),点B(x2,y2)を端点とする線分と点p(x,y)との距離を計算するプログラム
// double distance_between_points(double x1, double y1, double x2, double y2, double x, double y) {
//     //ABの長さを求める
//     double length_AB = std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
//     // cout << std::pow((x2 - x1), 2) << endl;
//     cout << "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2 << " x: " << x << " y: " << y << endl; 
//     cout << "length_AB " << length_AB << endl;
    
//     // ab・ap, ba・bp（内積）を求める
//     double AB_dot_AP = dot(x2-x1, y2-y1, x-x1, y-y1); //AB・APの内積を求める
//     cout << "AB_dot_AP " << AB_dot_AP << endl;
//     double BA_dot_BP = dot(x1-x2, y1-y2, x-x2, y-y2); //BA・BPの内積を求める
//     cout << "BA_dot_BP " << BA_dot_BP << endl;
//     // Calculate the cross product of vectors AP and AB：外積の計算
//     double AP_cross_product_AB = cross_product(x-x1, y-y1, x2-x1, y2-y1);       
//     cout << "AP_cross_product_AB " << AP_cross_product_AB << endl;

//     if (AB_dot_AP < 0.0){ // AB・APが負なら点Aの外側なので、点Aと点Pの距離を求める
//         cout << "ab・apが負です" << endl;
//         return std::sqrt(std::pow((x - x1), 2) + std::pow((y - y1), 2));
//     }
//     else if (BA_dot_BP < 0.0){
//         cout << "bA・Bpが負です" << endl;
//         return std::sqrt(std::pow((x - x2), 2) + std::pow((y - y2), 2));
//     }
//     else{
//         return std::abs(AP_cross_product_AB)/ length_AB;
//     }

// }

// ------------------------尤度関数------------------------------
// 白線ポリゴンにおけるマッチングによる尤度計算を行う関数
void polygonMatchingLikelihood(const pcl::PointCloud<pcl::PointXYZ>& downsampled_cloud,
                               const geometry_msgs::PoseArray& particle_cloud,
                               double global_initial_x,double global_initial_y, double global_initial_yaw,
                               double time_stamp,
                               std::ofstream& ofs) {
    /*
        引数：
        　・downsampleされたpointcloud→line_cloudにすればダウンサンプル無しの点群を受け取るようにできる
        　・particle_cloud：arrowそれぞれの座標
        　・time_stamp：時間
        返り値：
        　・particle_value
    */

    // 尤度計算
    double line_vertical_small = 0;
    double line_vertical_big = 0;
    double line_side_small = 0;
    double line_side_big = 0;
    double likelihood_value = 0;
    // 処理を実装する
    for (int i = 0; i < PARTICLE_NUM; i++) // パーティクル一個一個について計算していく
    {
        double roll, pitch, yaw;
        geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); // yaw角に変換
        int match_count = 0;                                                         // 尤度のマッチ数
        // 一箇所の尤度を調べるというv,wが動かない前提ならば、
        // 初期値一つのグローバル座標を与えてあげるだけで良いので余計な変換はいらない
        
        for (int j = 0; j < downsampled_cloud.points.size(); j++) // 検出した白線の座標配列
        {

            // particle一つ一つについて絶対座標に変換(lineがjでtheta,pointsがiに注意)
            double line_glx = downsampled_cloud.points[j].x * cosf(yaw) - downsampled_cloud.points[j].y * sinf(yaw) + particle_cloud.poses[i].position.x;
            double line_gly = downsampled_cloud.points[j].x * sinf(yaw) + downsampled_cloud.points[j].y * cosf(yaw) + particle_cloud.poses[i].position.y;
            // std::cout << "particle_x :" << particle_cloud.poses[i].position.x << " " << "particle_y" << particle_cloud.poses[i].position.y << " " << "particle_yaw" << yaw << std::endl;
            // std::cout << "prticle_num :" << i << " " <<"line_num :" << j << " "<< "line_glx: " << line_glx << " " << "line_gly" << line_gly << std::endl;
            for (int k = 0; k < POLYGON_NUM; k++)
            {
                // 上下の小さい方か大きいほうかを決める
                if (WHITE_POLYGON_MAP[k][0] < WHITE_POLYGON_MAP[k][1])
                {
                    line_vertical_small = WHITE_POLYGON_MAP[k][0];
                    line_vertical_big = WHITE_POLYGON_MAP[k][1];
                }
                else
                {
                    line_vertical_small = WHITE_POLYGON_MAP[k][1];
                    line_vertical_big = WHITE_POLYGON_MAP[k][0];
                }
                // 左右の小さい方か大きい方か決める
                if (WHITE_POLYGON_MAP[k][2] < WHITE_POLYGON_MAP[k][3])
                {
                    line_side_small = WHITE_POLYGON_MAP[k][2];
                    line_side_big = WHITE_POLYGON_MAP[k][3];
                }
                else
                {
                    line_side_small = WHITE_POLYGON_MAP[k][3];
                    line_side_big = WHITE_POLYGON_MAP[k][2];
                }
                // std::cout << line_vertical_small << " " << line_glx << " " << line_gly << std::endl;
                if (line_vertical_small < line_glx && line_glx < line_vertical_big && line_side_small < line_gly && line_gly < line_side_big)
                {
                    match_count++;
                    break;
                }
                
            }

            // std::cout << "マッチ数" << match_count << std::endl;
        }

        likelihood_value = double(match_count) / double(downsampled_cloud.points.size());
        // std::cout <<"マッチ数:" << match_count << " " <<  "白線の全部:"<< line_posi.points.size() <<" "<<  "尤度:" << std::fixed << std::setprecision(10) << likelihood_value << std::endl;
        particle_value[i] *= likelihood_value; // 前回の重みｘ尤度＝今回の重み
        std::cout << "尤度" << likelihood_value << std::endl;


        ofs << time_stamp << ", " << particle_cloud.poses[i].position.x << ", " << particle_cloud.poses[i].position.y << ", " << yaw << ", " << particle_value[i] << endl;
    
    }

}

// void polygonMatchingLikelihood_crossProduct(const pcl::PointCloud<pcl::PointXYZ>& downsampled_cloud,
//                                const geometry_msgs::PoseArray& particle_cloud,
//                                double global_initial_x,double global_initial_y, double global_initial_yaw,
//                                double time_stamp,
//                                std::ofstream& ofs) {
                                
    
//                                }
// // 白線の中心の線分と白線の観測点群との点と直線との距離から求める尤度関数
// void pointCenterLineDistanceLikelihood(const pcl::PointCloud<pcl::PointXYZ>& downsampled_cloud,
//                                         const geometry_msgs::PoseArray& particle_cloud,
//                                         double time_stamp,
//                                         std::ofstream& ofs){
    
//     double likelihood_value = 0.0;
//     double minDist = 100000; // 大きい値を初期値にする
    
//     // 仮想的な白線点群で実験
//     double white_line_px[3] = {1,   10,    11};
//     double white_line_py[3] = {0.4, -0.55, 3.5};

//     // 処理を実装する
//     // for (int i = 0; i < PARTICLE_NUM; i++) // パーティクル一個一個について計算していく
//     // {
//     //     double roll, pitch, yaw;
//     //     geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); // yaw角に変換

//     //     double points_score = 0.0;

        
//         // for (int j = 0; j < downsampled_cloud.points.size(); j++) // 検出した白線の座標配列
//     for (int j = 0; j < 3; j++)
//     {
//         // // particle一つ一つについて絶対座標に変換(lineがjでtheta,pointsがiに注意)
//         // double white_point_glx = downsampled_cloud.points[j].x * cosf(yaw) - downsampled_cloud.points[j].y * sinf(yaw) + particle_cloud.poses[i].position.x;
//         // double white_point_gly = downsampled_cloud.points[j].x * sinf(yaw) + downsampled_cloud.points[j].y * cosf(yaw) + particle_cloud.poses[i].position.y;

//         // 仮想的な白線点群3つで実験
//         double white_point_glx = white_line_px[j];
//         double white_point_gly = white_line_py[j];
        
//         for (int k = 0; k < POLYGON_NUM; k++)
//         {
//             double x1 = WHITE_CENTERLINE_COODINATE[k][0];
//             double y1 = WHITE_CENTERLINE_COODINATE[k][1];
//             double x2 = WHITE_CENTERLINE_COODINATE[k][2];
//             double y2 = WHITE_CENTERLINE_COODINATE[k][3];

//             double distance = distance_between_points(x1, y1, x2, y2, white_point_glx, white_point_gly);
            
//             if (minDist > distance)
//             {
//                 minDist = distance; //一つの点群と一番近いポリゴンとの距離を決める
//             }

//         }

//         //     points_score += minDist; // 全点群の最近線分の距離の和を算出

//         // }

//         // parti

//         // likelihood_value = double(points_score) / double(downsampled_cloud.points.size());

//         // likelihood_value = double(match_count) / double(downsampled_cloud.points.size());
//         // std::cout <<"マッチ数:" << match_count << " " <<  "白線の全部:"<< line_posi.points.size() <<" "<<  "尤度:" << std::fixed << std::setprecision(10) << likelihood_value << std::endl;
        
//         // particle_value[i] *= likelihood_value; // 前回の重みｘ尤度＝今回の重み


//         // ofs << time_stamp << ", " << particle_cloud.poses[i].position.x << ", " << particle_cloud.poses[i].position.y << ", " << yaw << ", " << particle_value[i] << endl;
    
//     }

// }



// メイン関数
int main(int argc, char **argv)
{

    ros::init(argc, argv, "likelifood_distribution");
    ros::NodeHandle nh;
    
    // コマンドライン引数を取得-------------------------------------------------
    double param_global_init_x, param_global_init_y, param_global_init_yaw;
    if (argc == 4)
    {
        param_global_init_x   = std::stod(argv[1]); // 文字列→doubleに変換
        param_global_init_y   = std::stod(argv[2]);
        param_global_init_yaw = std::stod(argv[3]);
    }
    else
    {
        ROS_WARN("引数が指定されていません。デフォルト値を使用します。");
        param_global_init_x   = 10.7;
        param_global_init_y   = 3.0;
        param_global_init_yaw = -1.5708;
    }
    // -----------------------------------------------------------------------------

    // 配置したいarrowの中心を受け取る（x,y,θ)
    double global_initial_x   = param_global_init_x;
    double global_initial_y   = param_global_init_y;
    double global_initial_yaw = param_global_init_yaw; // rad


    // // 点と線分の距離が正しいかどうかを確認するデバッグ用
    // double ax = 0, ay = 0;
    // double bx = 2, by = 0;
    // double px = 4, py = 0;
    // double result = distance_between_points(ax,ay, bx, by, px, py);
    // cout << "result " << result << endl;

    // // // パーティクルの座標の保存用csvファイル------------------------------------------------------gazebo_downsampled_ or gazebo_nashi
    std::stringstream ss;
    ss << "/home/hirayama-d/research_ws/src/sim/ICMRE2024_csv/20230905_boxel_nasi_2.0ikanomi_" << std::fixed << std::setprecision(1) << param_global_init_x << "_"<< std::fixed << std::setprecision(1) << param_global_init_y << "_" << std::fixed << std::setprecision(2) << param_global_init_yaw <<".csv";
    std::string file_name = ss.str();
    std::ofstream ofs(file_name); // 任意のファイル名を生成する

    // -----------------------------------------------------------------------------

    // Publisher------------------------------------------------------------------------------------
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5);
    // Pub:確認用
    // ros::Publisher line_global_points_pub = nh.advertise<sensor_msgs::PointCloud>("global_line", 5);

    // Subscriber------------------------------------------------------------------------------------------
    // ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);
    ros::Subscriber line_points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front_camera/line_points_1", 1, lineCloudCallback);
    ros::Subscriber line_downsample_points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front_camera/down_sample_line_points2", 1, pointCloudCallback);
    tf::TransformBroadcaster broadcaster; //particle_arrayのブロードキャスト

    ros::Rate rate(10.0);
    int cnt = 0;


    particle_cloud.poses.resize(PARTICLE_NUM); // particle_num

    // 中心座標とサイズを指定（初期姿勢位置）
    pair<double, double> center = make_pair(global_initial_x, global_initial_y);
    int size = 20; // particle_num = size * size
    double width = 0.5;

    // 中心座標を中心とした二次元座標配列の生成
    vector<vector<pair<double, double>>> coordinates = generateCoordinateArray(center, size, width);

    // 二次元座標配列の内容を表示
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            cout << "(" << coordinates[i][j].first << ", " << coordinates[i][j].second << ") ";
        }
        cout << endl;
    }

    // 初期のparticleの位置と姿勢を座標から取得
    int pn = 0; // pnはmaxでparticle_num // particle_numはsize*size
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            particle_cloud.poses[pn].position.x = coordinates[i][j].first;
            particle_cloud.poses[pn].position.y = coordinates[i][j].second;
            particle_cloud.poses[pn].orientation = rpy_to_geometry_quat(0, 0, global_initial_yaw); //(rall, pitch, yaw)= yawが0度
            particle_value[pn] = 1.0 / double(PARTICLE_NUM);                                       // 最初の重み（均等にする）
            pn++;
        }
    }
    // pointCenterLineDistanceLikelihood(downsampled_cloud, particle_cloud, time_stamp, ofs);
    

    while (ros::ok())
    {
        ros::spinOnce();
        time_stamp += dt; // dtで時間を積算



        if (downsampled_cloud.points.size() > 0)
        {
            // 白線ポリゴンにおけるマッチングによる尤度関数&csvに保存（downsampled_cloud→line_cloud　変更可能)
            polygonMatchingLikelihood(downsampled_cloud, particle_cloud,global_initial_x, global_initial_y, global_initial_yaw, time_stamp, ofs);
            
            // 白線の中心の線分と白線の観測点群との点と直線との距離から求める尤度関数＆csvに保存
            // pointCenterLineDistanceLikelihood(downsampled_cloud, particle_cloud, time_stamp, ofs);
        }

    //  // if (cnt== 1)
        // {
        //     break;
        // }
        // cnt++;

        // publish
        // particle_cloud.header.frame_id = "beego/odom";
        // particle_cloud.header.stamp = ros::Time::now();
        
        // 白線点群
        tf::StampedTransform transform_particle_pose;
        transform_particle_pose.setOrigin(tf::Vector3(global_initial_x, global_initial_y, 0));
        transform_particle_pose.setRotation(getYawQuaternion(global_initial_yaw));
        // ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
        transform_particle_pose.stamp_ = ros::Time::now();
        transform_particle_pose.frame_id_ = "beego/odom";
        transform_particle_pose.child_frame_id_ = "fixed_paritcle_posi";
        broadcaster.sendTransform(transform_particle_pose);

        // geometry_msgs::PoseArray particle_cloud;
        particle_cloud.header.frame_id = "beego/odom";
        particle_cloud_pub.publish(particle_cloud);
        // line_global_points_pub.publish(line_posi);
    }
    // rate.sleep();
}