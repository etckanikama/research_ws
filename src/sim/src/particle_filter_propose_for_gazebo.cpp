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
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <time.h>
#include <chrono>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gazebo_msgs/ModelStates.h>
#include <limits>
#include <cstdlib>  // getenvを使うために必要
#include <sstream> // std::ostringstreamを使用するために必要

// シミュレータ環境におけるパーティクルフィルタを用いた位置推定の検証
/*
・現gazebo環境において位置推定アルゴリズムが動くようにする
    ・ちゃんとあってるかどうかの確認方法は？
    ・gazebo環境におけるカメラのキャリブレーションがあわせる
    ・ジョイコンでの走行データをrosbagで記録→実験

・マップの拡張を行う
    ・白線地図を作り直す
    ・ポリゴンを定義し直してworldも作り直す
*/

using namespace std;




ros::Time point_timestamp; //初期化
sensor_msgs::PointCloud line_posi;
sensor_msgs::PointCloud2 down_sample_line_posi;
geometry_msgs::PoseArray particle_cloud; // poseArrayの値の入れ方：https://answers.ros.org/question/251586/rostopic-pub-geometry_msgsposearray-example/
pcl::PointCloud<pcl::PointXYZ> downsampled_cloud, line_cloud;
geometry_msgs::PoseWithCovarianceStamped estimate_posi,max_estimate_posi;
geometry_msgs::Twist cmd_vel;

// 軌跡の保存用csvファイル
// std::ofstream ofs("ccc.csv");
double beego_x, beego_y, beego_z, beego_yaw;
double beego_orientation_x, beego_orientation_y, beego_orientation_z, beego_orientation_w;
double beego_linear_x, beego_angular_z;
bool beego_pose_available = false;


// 初期速度指令値
double v = 0.0;//joyconから得られるliniear.xなどによって更新はされる
double omega = 0.0;

// const double DT = 1.0 / 10.0;
double DelayThreshold = 0.2; //点群の遅延を許容する計算性能

double robot_x = 0.0; //オドメトリによるx座標
double robot_y = 0.0; //オドメトリによるy座標
double robot_yaw; // オドメトリによるyaw

// 尤度更新の状態→デフォルトしない(0),する(1)
int likelihood_state = 0;


// // 乱数生成エンジン
std::random_device seed;
std::mt19937 engine(seed());  

// // 実機のパーティクルに付与するノイズ
double sigma_v_v = 0.1;  // 直進1mで生じる距離の標準偏差0.03
double sigma_omega_v = 0.1; // 直進1mで生じる回転の誤差0.001
double sigma_v_omega = 0.1;  //回転1radで生じる直進の誤差0.05
double sigma_omega_omega = 0.3;    //回転1radで生じる回転の誤差0.1

std::normal_distribution<> dist_v_v(0.0, sigma_v_v);
std::normal_distribution<> dist_omega_v(0.0, sigma_omega_v);
std::normal_distribution<> dist_v_omega(0.0, sigma_v_omega);
std::normal_distribution<> dist_omega_omega(0.0, sigma_omega_omega);


// // パーティクル数
const int PARTICLE_NUM  = 500; 
double coeff = 0.01; //尤度係数：実機0.1,⇛gazebo:0.01
double beta = 2.0; // 改良の尤度計算の係数
double Mth = 300; //最低限含まれてほしい点群数のしきい値    
double sigma_px_th =0.15;  //x方向の標準偏差のしきい値:白線の幅(0.075)*2=0.15
double sigma_py_th =0.15; //y方向の標準偏差のしきい値:白線の幅(0.075)*2=0.15
// double avg = 0.0, sig = 0.01; //平均0,標準偏差0.3(初期の散らばり具合)

std::vector<double> particle_value(PARTICLE_NUM); // particleの重み
double theta_pt[PARTICLE_NUM]= {0.0};// // パーティクルの姿勢

// // // 白線のポリゴン情報マップ(x_bottom, x_top, y_right, y_left)
// const int POLYGON_NUM = 40 ; // 通常マップ：25, 1.5ひげマップ:40, 統合ひげ:37
// double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 12.6, 1.76, 1.835}, {11.41, 12.6, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.61}, {10.02, 10.095, -0.545, -3.61}, {0, 10.095, -0.47, -0.545},
//                                             {0.47, 0.545, -1.465, -3.61},{0.545, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
//                                             {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61},

//                                             // 統合ひげ追加
//                                             // {4.174038461538462, 4.249038461538461, 0.20750000000000007, 0.8075000000000001}, {1.2660714285714287, 1.341071428571429, 0.20750000000000007, 0.8075000000000001}, {-0.0375, 0.0375, 0.20750000000000007, 0.8075000000000001}, {9.4115, 9.4865, 1.5975, 2.1975}, {8.54325, 8.61825, -0.8075000000000001, -0.20750000000000007}, {7.2309, 7.305899999999999, -0.8075000000000001, -0.20750000000000007}, {5.7166500000000005, 5.79165, -0.8075000000000001, -0.20750000000000007}, {2.68815, 2.76315, -0.8075000000000001, -0.20750000000000007}, {11.1475, 11.7475, -0.1979651162790698, -0.12296511627906978}, {11.1475, 11.7475, -2.72703488372093, -2.6520348837209298}, {9.7575, 10.357500000000002, -1.499, -1.4240000000000002}, {9.7575, 10.357500000000002, -3.6375, -3.5625}
//                                             // 1.5のひげ
//                                             {9.757499999999999, 10.3575, -3.5825, -3.5075}, {2.9625, 3.0375, 0.20749999999999996, 0.8074999999999999}, {5.9625, 6.0375, -0.8074999999999999, -0.20749999999999996}, {2.9625, 3.0375, -0.8074999999999999, -0.20749999999999996}, {7.4625, 7.5375, -0.8074999999999999, -0.20749999999999996}, {1.4625, 1.5375, 0.20749999999999996, 0.8074999999999999}, {1.4625, 1.5375, -0.8074999999999999, -0.20749999999999996}, {11.175, 11.775, -0.7875, -0.7125}, {11.175, 11.775, -2.2875, -2.2125}, {9.757499999999999, 10.3575, -2.0825, -2.0075}, {8.9625, 9.0375, -0.8074999999999999, -0.20749999999999996}, {5.9625, 6.0375, 0.20749999999999996, 0.8074999999999999}, {7.4625, 7.5375, 0.20749999999999996, 0.8074999999999999}, {4.4625, 4.5375, 0.20749999999999996, 0.8074999999999999}, {4.4625, 4.5375, -0.8074999999999999, -0.20749999999999996}
//                                             }; 

// 実際の多角形数を格納する変数
int POLYGON_NUM;

const int MAX_POLYGON_NUM = 40; // 最大の多角形数
double WHITE_POLYGON_MAP[MAX_POLYGON_NUM][4];

const int ORIGIN_POLYGON_NUM = 25; //origin_map
double origin_map[ORIGIN_POLYGON_NUM][4]     = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 12.6, 1.76, 1.835}, {11.41, 12.6, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.61}, {10.02, 10.095, -0.545, -3.61}, {0, 10.095, -0.47, -0.545},
                                            {0.47, 0.545, -1.465, -3.61},{0.545, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
                                            {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61},
                                            }; 

const int HIHUKU_POLYGON_NUM = 37; //hihuku_map
double hihuku_map[HIHUKU_POLYGON_NUM][4]     = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 12.6, 1.76, 1.835}, {11.41, 12.6, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.61}, {10.02, 10.095, -0.545, -3.61}, {0, 10.095, -0.47, -0.545},
                                            {0.47, 0.545, -1.465, -3.61},{0.545, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
                                            {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61},
                                            // 統合ひげ追加
                                            {4.174038461538462, 4.249038461538461, 0.20750000000000007, 0.8075000000000001}, {1.2660714285714287, 1.341071428571429, 0.20750000000000007, 0.8075000000000001}, {-0.0375, 0.0375, 0.20750000000000007, 0.8075000000000001}, {9.4115, 9.4865, 1.5975, 2.1975}, {8.54325, 8.61825, -0.8075000000000001, -0.20750000000000007}, {7.2309, 7.305899999999999, -0.8075000000000001, -0.20750000000000007}, {5.7166500000000005, 5.79165, -0.8075000000000001, -0.20750000000000007}, {2.68815, 2.76315, -0.8075000000000001, -0.20750000000000007}, {11.1475, 11.7475, -0.1979651162790698, -0.12296511627906978}, {11.1475, 11.7475, -2.72703488372093, -2.6520348837209298}, {9.7575, 10.357500000000002, -1.499, -1.4240000000000002}, {9.7575, 10.357500000000002, -3.6375, -3.5625}
                                            }; 

const int ONE_FIVE_POLYGON_NUM = 40;
double one_five_map[ONE_FIVE_POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 12.6, 1.76, 1.835}, {11.41, 12.6, 0.75, 0.825}, {11.41, 11.485, 0.75, -3.61}, {10.02, 10.095, -0.545, -3.61}, {0, 10.095, -0.47, -0.545},
                                            {0.47, 0.545, -1.465, -3.61},{0.545, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
                                            {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61},
                                            // 1.5のひげ
                                            {9.757499999999999, 10.3575, -3.5825, -3.5075}, {2.9625, 3.0375, 0.20749999999999996, 0.8074999999999999}, {5.9625, 6.0375, -0.8074999999999999, -0.20749999999999996}, {2.9625, 3.0375, -0.8074999999999999, -0.20749999999999996}, {7.4625, 7.5375, -0.8074999999999999, -0.20749999999999996}, {1.4625, 1.5375, 0.20749999999999996, 0.8074999999999999}, {1.4625, 1.5375, -0.8074999999999999, -0.20749999999999996}, {11.175, 11.775, -0.7875, -0.7125}, {11.175, 11.775, -2.2875, -2.2125}, {9.757499999999999, 10.3575, -2.0825, -2.0075}, {8.9625, 9.0375, -0.8074999999999999, -0.20749999999999996}, {5.9625, 6.0375, 0.20749999999999996, 0.8074999999999999}, {7.4625, 7.5375, 0.20749999999999996, 0.8074999999999999}, {4.4625, 4.5375, 0.20749999999999996, 0.8074999999999999}, {4.4625, 4.5375, -0.8074999999999999, -0.20749999999999996}
                                            }; 


// roll, pitch, yawからクォータニオンにする関数
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

// クォータニオンをオイラーに変換
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}



// // cmd_velとodomの値をcallback
void velCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    v = msg->twist.twist.linear.x;
    omega = msg->twist.twist.angular.z;
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_yaw); // Yaw 角を取得

}

// // cbPoint
void line_point_cb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    line_posi.points = msg->points;
}

// ボクセルダウンサンプリングをしたあとの点群を読み込む
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // PointCloud2メッセージをPCLのPointCloud型に変換する
    point_timestamp = msg->header.stamp;

    pcl::fromROSMsg(*msg, downsampled_cloud);

}

// modelStatesCallback関数
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int beego_index = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "beego") {
            beego_index = i;
            break;
        }
    }

    if (beego_index != -1) {
        beego_x = msg->pose[beego_index].position.x;
        beego_y = msg->pose[beego_index].position.y;
        beego_z = msg->pose[beego_index].position.z;
        // cout << "beego_xxxxxxxxxxxxxxxxxxx:" << " " << beego_x << endl;
        beego_orientation_x = msg->pose[beego_index].orientation.x;
        beego_orientation_y = msg->pose[beego_index].orientation.y;
        beego_orientation_z = msg->pose[beego_index].orientation.z;
        beego_orientation_w = msg->pose[beego_index].orientation.w;
        tf::Quaternion q(
            beego_orientation_x,
            beego_orientation_y,
            beego_orientation_z,
            beego_orientation_w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        beego_yaw = yaw;


        // linear.x と angular.z の値を取得
        beego_linear_x = msg->twist[beego_index].linear.x;
        beego_angular_z = msg->twist[beego_index].angular.z;

        beego_pose_available = true;
    }
}

// パーティクル数で割って重みの正規化（重みの合計が1になるようにしてる）
void likelihood_value_nomalization()
{
    double sum = 0;
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        sum += particle_value[i];
    }
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        
        if (particle_value[i] != 0)
        {
            particle_value[i] = double(particle_value[i]) / double(sum); 
        }

    }
}


// リサンプリングの処理
void resampling()
{

    double rand_width = 1.0 / double(PARTICLE_NUM); // 乱数幅
    std::uniform_real_distribution<float> distr(0, rand_width); // 0~rand_widthの範囲で当確率で発生する
    double rand = distr(engine); // 積み上げ乱数
    double w_sum = particle_value[0]; //積み上げ正規化重み
    int n_before = 0; // リサンプリング前の重みのインデックス
    int n_after = 0; // リサンプリング後の重みのインデックス

    
    geometry_msgs::PoseArray particle_new; // リサンプリング後の粒子
    std::vector<double> particle_value_new(PARTICLE_NUM); //リサンプリング後の粒子の重み
    particle_new.poses.resize(PARTICLE_NUM); 

    // 要素のコピー
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        particle_new.poses[i] = particle_cloud.poses[i];
        particle_value_new[i] = particle_value[i];
    }

    // 系統リサンプリングの処理開始
    while (n_after < PARTICLE_NUM)
    {
        if (rand > w_sum) //乱数が重みより大きければ重みを積む
        {
            n_before++;
            w_sum += particle_value[n_before];
        }
        else // 乱数が重み以下ならば、リサンプリングし、乱数を積む 
        {

            particle_new.poses[n_after] = particle_cloud.poses[n_before];
            particle_value_new[n_after] = particle_value[n_before];
            rand += rand_width; //乱数を積む
            n_after++;
        }

    }

    // 元のパーティクルに代入
    for (int i=0; i < PARTICLE_NUM; i++)
    {
        particle_cloud.poses[i] = particle_new.poses[i];
        particle_value[i] = 1.0 / double(PARTICLE_NUM); // 重みの初期化
    }

}



// // メイン
int main(int argc, char **argv)
{

    ros::init(argc, argv, "particle_node_gazebo");
    ros::NodeHandle nh;

    // ----------------------------環境変数の設定：csv保存パスの設定-------------------------
    // 環境変数からホームディレクトリのパスを取得
    char* homeDir = getenv("HOME");
    if (homeDir == nullptr) {
        std::cerr << "ホームディレクトリが見つかりません。" << std::endl;
        return 1;
    }
    // コマンドライン引数の検証
    if (argc != 3) {
        std::cerr << "引数の数が正しくありません。" << std::endl;
        return 1;
    }

    std::string arg1 = argv[1]; // 1つ目の引数
    std::string arg2 = argv[2]; // 2つ目の引数

    // 1つ目の引数が有効な候補かチェック
    if (arg1 != "origin" && arg1 != "1.5" && arg1 != "hihuku") {
        std::cerr << "1つ目の引数が無効です。" << std::endl;
        return 1;
    }

    // 2つ目の引数が有効な候補かチェック
    if (arg2 != "1" && arg2 != "2" && arg2 != "3" && arg2 != "4" && arg2 != "5") {
        std::cerr << "2つ目の引数が無効です。" << std::endl;
        return 1;
    }
    // ファイルパスを構築
    std::string folderPath1 = std::string(homeDir) + "/research_ws/src/sim/src/particle_result_csv/" + arg1 + "_map";
    std::string filepath1 = folderPath1 + "/" + arg2 + ".csv";
    
    // ファイルパスを構築
    // std::string filepath1 = std::string(homeDir) + "/research_ws/src/sim/src/particle_result_csv/result.csv";
    std::string filepath2 = std::string(homeDir) + "/research_ws/src/sim/src/particle_result_csv/v_ω.csv";

    // 以降、ファイル操作は変更なし
    std::ofstream ofs1(filepath1, std::ios::out | std::ios::trunc);
    std::ofstream ofs2(filepath2, std::ios::out | std::ios::trunc);
    // ヘッダーの追加
    ofs1 << "Count" << "," <<"TimeStamp" << "," << "RobotX" << "," << "RobotY" << "," << "RobotYaw" << "," << "BeegoX" << "," << "BeegoY" << "," << "BeegoYaw"
    << "," << "EstimateOdomX"<< "," << "EstimateOdomY"<< "," << "EstimateOdomYaw"<< "," << "LikelihoodState" << "," << "MaxLikelihood" << "," << "SumLikelihood"
    << "," << "index" << "," << "ParticlePosX_" << "," << "ParticlePosY_" << "," << "ParticlePosYAW_" << "," << "ParticleLikelihood_" 
    << "," << "input_V" << "," << "input_ω" << "," << "Px-BeegoX" << "," << "Py-BeegoY" << ","  <<"Pyaw-BeegoYaw" 
    << "," << "RobotX-BeegoX" << "," << "RobotY-BeegoY" << "," << "RobotYaw-BeegoYaw" << "," << "EstimateOdomX-BeegoX" << "," << "EstimateOdomY-BeegoY" 
    << "," << "EstimateOdomYaw-BeegoYaw" << endl;
    // -------------------------------------------------------------------------    

    // -----------------------------コマンドライン引数に従ってマップを変更する------------
    double (*source_map)[4];
    int source_polygon_num;

    if (arg1 == "origin") {
        source_map = origin_map;
        source_polygon_num = ORIGIN_POLYGON_NUM;
        cout << "origin_mapを使用中" <<endl;
    } else if (arg1 == "1.5") {
        source_map = one_five_map;
        source_polygon_num = ONE_FIVE_POLYGON_NUM;
        cout << "1.5_mapを使用中" <<endl;
    } else if (arg1 == "hihuku") {
        source_map = hihuku_map;
        source_polygon_num = HIHUKU_POLYGON_NUM;
        cout << "hihuku_mapを使用中" <<endl;
    } else {
        ROS_WARN("Invalid map type: %s. Using default map.", arg1.c_str());
        source_map = origin_map;  // デフォルトの配列を使用
        source_polygon_num = ORIGIN_POLYGON_NUM;
    }

    // 選択されたマップのポリゴン数を更新
    POLYGON_NUM = source_polygon_num;
    cout << "polygon_num " << POLYGON_NUM << endl;

    // 選択された配列の内容をWHITE_POLYGON_MAPにコピー
    std::copy(&source_map[0][0], &source_map[0][0] + source_polygon_num * 4, &WHITE_POLYGON_MAP[0][0]);

    // ---------------------------------- Mapが変更されているはず

    
    if (POLYGON_NUM==40)
    {
        std::cout << "gazebo 1.5 map"<< std::endl;

    }
    if (POLYGON_NUM==37)
    {
        std::cout << "gazebo set covering map"<< std::endl;

    }
    if (POLYGON_NUM==25)
    {
        std::cout << "gazebo origin map"<< std::endl;

    }
//     // Publisher
    ros::Publisher self_localization_pub = nh.advertise<nav_msgs::Odometry>("self_position", 10); // robotにモータに指令値を送るpublisherの宣言。
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5); // リサンプリング後のパーティクル
    ros::Publisher esitimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate_position",5);// posewithcoverianceで推定値をpublish
    ros::Publisher max_estimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("max_estimate_position",5);// posewithcoverianceで推定値をpublish
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("beego_pose_cov", 10); //これはgazeboのmodel_stateです

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/beego/diff_drive_controller/odom",10, velCallback);//gazeboを挟んだオドメトリ/beego/diff_drive_controller/odom をサブスクライブ
    ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);
    ros::Subscriber line_downsample_points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front_camera/down_sample_line_points2", 1, pointCloudCallback);
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);



    
    nav_msgs::Odometry posi;
    tf::TransformBroadcaster broadcaster;
    
    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;// rosbagの初期値にしたがって変更

    


    posi.header.frame_id = "map";
    estimate_posi.header.frame_id = "map";
    max_estimate_posi.header.frame_id = "map";
    particle_cloud.header.frame_id = "map";
    particle_cloud.header.stamp = ros::Time::now();
    particle_cloud.poses.resize(PARTICLE_NUM);

    
    // 初期のparticleの位置と姿勢
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        //ガウス分布（平均、標準偏差）
        std::normal_distribution<> initial_x_distribution(init_x, 0.1); //sigmaをいじれば初期のパーティクルのばらつきが決まる
        std::normal_distribution<> initial_y_distribution(init_y, 0.1);
        std::normal_distribution<> initial_theta_distribution(init_yaw, 0.1); //姿勢の散らばり

        particle_cloud.poses[i].position.x = initial_x_distribution(engine);
        particle_cloud.poses[i].position.y = initial_y_distribution(engine);
        particle_cloud.poses[i].position.z = 0.0;
        particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0, 0, initial_theta_distribution(engine)); 
        particle_value[i] = 1.0 / double(PARTICLE_NUM); //最初の重みは均一


    }



    ros::Rate rate(10.0);
    int cnt = 0;
    double time_stamping = 0.0;
    ros::Time start= ros::Time::now();
    std::cout << "Start time: " << start.toSec() << std::endl;

    while (ros::ok())
    {
        likelihood_state = 0; // 尤度更新はデフォルトはしない

        ros::Time current = ros::Time::now();
        ros::Duration elapsed = current - start;
        double DT = elapsed.toSec();
        if (DT==0 || DT > 130) DT =  1.0 / 10.0; //最初だけ0と意味わからん数値が入るやつを防ぐ

        
        double max_likelyhood = 0; // 最大尤度
        double sum_likelyhood = 0; //尤度の総和


        // double DT = 1 / s;  
        time_stamping += DT; // dtで時間を積算
        cout << "時間 " << time_stamping << endl;

        ros::spinOnce();


        double likelyhood_array[PARTICLE_NUM]; //尤度を格納する配列

        //----------------------------- パーティクルの位置に状態遷移モデルの適用------------------------------------------------------------
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            // ノイズをのせたvとomegaを生成
            double v_noise,omega_noise;
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            // v_noise = beego_linear_x + dist_v_v(engine) * sqrt(abs(beego_linear_x)/DT) + dist_v_omega(engine) * sqrt(abs(beego_angular_z)/DT); //gazebo_model_sateのv,ω
            v_noise = v + dist_v_v(engine) * sqrt(abs(v)/DT) + dist_v_omega(engine) * sqrt(abs(omega)/DT);
            omega_noise = omega + dist_omega_v(engine) * sqrt(abs(v)/DT) + dist_omega_omega(engine) * sqrt(abs(omega)/DT);
            // omega_noise = beego_angular_z + dist_omega_v(engine) * sqrt(abs(beego_linear_x)/DT) + dist_omega_omega(engine) * sqrt(abs(beego_angular_z)/DT);//gazebo_model_sateのv,ω

            particle_cloud.poses[i].position.x = particle_cloud.poses[i].position.x + v_noise * cos(yaw)*DT;
            particle_cloud.poses[i].position.y = particle_cloud.poses[i].position.y + v_noise * sin(yaw)*DT;
            particle_cloud.poses[i].position.z = 0.0; 
            particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0,0, yaw + omega_noise * DT);
        
        }
        
        // // 必ず真値にパーティクルをのせる（debug用）
        // particle_cloud.poses[0].position.x = beego_x;
        // particle_cloud.poses[0].position.y = beego_y;
        // particle_cloud.poses[0].orientation = rpy_to_geometry_quat(0,0, beego_yaw);
        
        // -----------------------------------------------------------------------------------------------------------------

        //---------------------------------------- 尤度の計算条件--------------------------------------------------------
        ros::Time now = ros::Time::now();// 現在時刻の取得と表示
        ros::Duration diff = now - point_timestamp;
        cout << "点群の遅延時間: " << diff.toSec() << " seconds" << endl;
        
        if (diff.toSec() < DelayThreshold) //遅延許容誤差：0.15以上の遅延では尤度計算を行わない
        {
            cout << "全白線点群数 " << downsampled_cloud.points.size() << endl;
            // 尤度計算
            // if (downsampled_cloud.points.size() > Mth) //最低限Mth以上含まれている時だけ尤度計算を行う
            // {
            //     // -------------標準偏差の計算----------------------
            //     // sumの作成
            //     double sum_px = 0.0, sum_py = 0.0;
            //     for (int j = 0; j < downsampled_cloud.points.size(); j++)
            //     {
            //         sum_px += downsampled_cloud.points[j].x;
            //         sum_py += downsampled_cloud.points[j].y;
            //     }
            //     // meanの作成
            //     double mean_px = sum_px / downsampled_cloud.points.size();
            //     double mean_py = sum_py / downsampled_cloud.points.size();
            //     //偏差の二乗
            //     double sumOfSquaredDeviation_x = 0.0;
            //     double sumOfSquaredDeviation_y = 0.0;
            //     for (int j = 0; j < downsampled_cloud.points.size(); j++)
            //     {
            //         double deviation_px = downsampled_cloud.points[j].x - mean_px;
            //         double deviation_py = downsampled_cloud.points[j].y - mean_py;
            //         sumOfSquaredDeviation_x += deviation_px * deviation_px;
            //         sumOfSquaredDeviation_y += deviation_py * deviation_py;
            //     }

            //     //標準偏差を算出
            //     double sigma_px = std::sqrt(sumOfSquaredDeviation_x / (downsampled_cloud.points.size() - 1));
            //     double sigma_py = std::sqrt(sumOfSquaredDeviation_y / (downsampled_cloud.points.size() - 1));
            //     // // -----------------------------------------------------
            //     // std::cout << downsampled_cloud.points.size() << std::endl;
            //     // std::cout << "x方向の標準偏差 " << sigma_px << " " << "y方向の標準偏差" << sigma_py << std::endl;

            //     if ((sigma_px > sigma_px_th) && (sigma_py > sigma_py_th)) //この条件を突破したら尤度計算が始まる
            //     {
            if (downsampled_cloud.points.size() > 0) // rsjの条件
            {
                likelihood_state = 1;// 尤度更新を行う状態にセット

                // 尤度計算用の変数
                double line_vertical_small = 0;
                double line_vertical_big   = 0;
                double line_side_small     = 0;
                double line_side_big       = 0;
                double likelihood_value    = 0;


                for (int i = 0; i < PARTICLE_NUM; i++) //パーティクルのループ
                {
                    double roll, pitch, yaw;
                    geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
                    int match_count = 0; //尤度のマッチ数
                    // int a_match_count = 0; //別の方法のマッチ数

                    for (int j = 0; j < downsampled_cloud.points.size(); j++)//尤度計算の中のループ
                    {
                        // particle一つ一つを使って白線点群を絶対座標に変換
                        double line_glx = downsampled_cloud.points[j].x * cosf(yaw) - downsampled_cloud.points[j].y * sinf(yaw) +  particle_cloud.poses[i].position.x;
                        double line_gly = downsampled_cloud.points[j].x * sinf(yaw) + downsampled_cloud.points[j].y * cosf(yaw) +  particle_cloud.poses[i].position.y;
                        
                        for (int k = 0; k < POLYGON_NUM; k++ ) 
                        {
                            // 上下の小さい方か大きいほうかを決める
                            if (WHITE_POLYGON_MAP[k][0] < WHITE_POLYGON_MAP[k][1])
                            {
                                line_vertical_small = WHITE_POLYGON_MAP[k][0];
                                line_vertical_big   = WHITE_POLYGON_MAP[k][1];
                            } 
                            else
                            {
                                line_vertical_small = WHITE_POLYGON_MAP[k][1];
                                line_vertical_big   = WHITE_POLYGON_MAP[k][0];
                            }
                            // 左右の小さい方か大きい方か決める
                            if (WHITE_POLYGON_MAP[k][2] < WHITE_POLYGON_MAP[k][3])
                            {
                                line_side_small = WHITE_POLYGON_MAP[k][2];
                                line_side_big   = WHITE_POLYGON_MAP[k][3];
                            } 
                            else
                            {
                                line_side_small = WHITE_POLYGON_MAP[k][3];
                                line_side_big   = WHITE_POLYGON_MAP[k][2];
                            }
                            // std::cout << line_vertical_small << " " << line_glx << " " << line_gly << std::endl;
                            if (line_vertical_small < line_glx && line_glx < line_vertical_big && line_side_small < line_gly && line_gly < line_side_big)
                            {
                                match_count++;
                                break;
                            }
                        }
                    }
                    // それぞれのmatchカウントの数
                    // cout << "全白線点群数 "<< downsampled_cloud.points.size() << " " <<"パーティクル_"<< i <<"のマッチ数 " << match_count << endl;

                    likelihood_value = (1 - coeff)*(double(match_count) / double(downsampled_cloud.points.size())) + coeff; // m/M今まで
                    // likelihood_value = (1 - coeff)*(double(match_count) / (beta*(double(downsampled_cloud.points.size()) - double(match_count)) + double(match_count))) + coeff;   // m
                    // // likelihood_value = 1 / (1 + PARTICLE_NUM * coeff) * (double(match_count) / double(downsampled_cloud.points.size())) + coeff;
                    particle_value[i] *= likelihood_value; //前回の重みｘ尤度＝今回の重み
                    likelyhood_array[i] = likelihood_value; //尤度配列に尤度を格納
                    // std::cout << likelihood_value << std::endl;
                }
            }
            //     }
            // }
        }
        // -------------------------------------1ループにおける尤度計算条件：終了--------------------------
        
        //-----------------------------1ループにおける尤度配列から最大尤度と尤度の総和を求める-----------------------------------------
        double maxValue = std::numeric_limits<double>::lowest();
        int maxIndex = 0;
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            // 最大の尤度を見つける
            if(likelyhood_array[i] > maxValue)
            {
                maxValue = likelyhood_array[i];
                maxIndex = i;
            }
            sum_likelyhood += likelyhood_array[i]; //純粋に尤度を位置から格納している：尤度の和
        }
        max_likelyhood = maxValue;
        // likelyhood_array[maxIndex] = 0.0; //尤度の格納配列を0で初期化⇛ループの最後で良い
        cout << "最大の尤度 " << max_likelyhood << endl;
        cout << "尤度の和 " << sum_likelyhood << endl;
        // --------------------------------------------------------------------------------------------------------------------------------

        // --------------------------------------膨張リセット---------------------------------------------
        // 一回のループにおける尤度が出る
        // 膨張リセットの判定:最大尤度が0.5以下なら膨張リセット
        // 膨張リセットの判定：尤度の合計が380以下なら膨張リセット
        // double th = 30; // デフォルトの閾値
        // static int rest_counter = 0;
        // if (cnt > 60)
        // {
        //     if (sum_likelyhood < th) 
        //     {
        //         rest_counter++; 
        //     }
        //     else
        //     {
        //         rest_counter = 0;
        //     }

        //     if (rest_counter >= 5)
        //     {
        //         cout << "------膨張リセット-----" << endl;
        //         for (int i = 0; i < PARTICLE_NUM; i++)
        //         {
        //             double roll, pitch, yaw;
        //             geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
        //             //ガウス分布（平均、標準偏差）
        //             std::normal_distribution<> initial_x_distribution(particle_cloud.poses[i].position.x, 0.01); //sigmaをいじれば初期のパーティクルのばらつきが決まる
        //             std::normal_distribution<> initial_y_distribution(particle_cloud.poses[i].position.y, 0.01);
        //             std::normal_distribution<> initial_theta_distribution(yaw, 0.01); //姿勢の散らばり

        //             particle_cloud.poses[i].position.x = initial_x_distribution(engine);
        //             particle_cloud.poses[i].position.y = initial_y_distribution(engine);
        //             particle_cloud.poses[i].position.z = 0.0;
        //             particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0, 0, initial_theta_distribution(engine)); 
        //             particle_value[i] = 1.0 / double(PARTICLE_NUM); //最初の重みは均一

        //         }
        //         rest_counter = 0;
        //     }
        // }
        // ------------------------------------------------------------------------------------------------------------------------


        // 重みの正規化 : particle_valueの値を正規化
        likelihood_value_nomalization();


        // 重みづけ平均の推定結果の自己位置の初期化-----------------------------------------------
        double estimate_odom_x = 0.0;
        double estimate_odom_y = 0.0;
        double estimate_theta  = 0.0;

        // // 重みづけ平均：自己位置を出す処理：自己位置 +＝ 重み[i] * (x,y,theta)[i]ーーーーーーーーーーーーーーーーーーーーー
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            estimate_theta  += particle_value[i] * yaw;

        }
        
        estimate_posi.pose.pose.position.x = estimate_odom_x;
        estimate_posi.pose.pose.position.y = estimate_odom_y;
        estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, estimate_theta);
        
        // ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

        // 最大値を尤度とする--------------------------------------------------------------------------------------------------
        double max_estimate_odom_x = 0.0;
        double max_estimate_odom_y = 0.0;
        double max_estimate_theta  = 0.0;
        double max_value = particle_value[0];
        int max_value_index = 0;
        for (int i = 0; i < PARTICLE_NUM; i++)
        {

            if (max_value < particle_value[i])
            {
                max_value = particle_value[i];
                max_value_index = i;
            }
        }

        double roll, pitch, yaw;
        geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[max_value_index].orientation); //yaw角に変換
        max_estimate_odom_x =   particle_cloud.poses[max_value_index].position.x;
        max_estimate_odom_y = particle_cloud.poses[max_value_index].position.y;
        max_estimate_theta  = yaw;

        max_estimate_posi.pose.pose.position.x = max_estimate_odom_x;
        max_estimate_posi.pose.pose.position.y = max_estimate_odom_y;
        max_estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, max_estimate_theta);
        // ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
        
        // 標準出力
        // std::cout << "paticle[0] x "<< particle_cloud.poses[0].position.x << " " <<"particle[0] y " << particle_cloud.poses[0].position.y << " " << "尤度 " <<likelyhood_array[0] << std::endl;
        std::cout << "真値 x " << beego_x << " " << "真値 y " << beego_y << " " <<"真値 yaw " << beego_yaw <<std::endl; 
        std::cout << "重み付け推定値 x " << estimate_odom_x << " "<< "重み付け推定値 y " << estimate_odom_y << std::endl;
        std::cout << "max推定値 x " << max_estimate_odom_x << " "<< "max推定値 y " << max_estimate_odom_y << std::endl;
        std::cout << "オドメトリ x " << robot_x << " "<< "オドメトリ y " << robot_y << std::endl;
        std::cout << "v  " << v << " " << "omega " << omega << std::endl;
        std::cout << "最大の重み " << particle_value[max_value_index] << std::endl;
        
        // csv出力
        // ofs1 << "-------------------------------------------------------------------------" << std::endl;

        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            double roll, pitch, particle_i_yaw;
            geometry_quat_to_rpy(roll, pitch, particle_i_yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            // cnt:ループ数
            ofs1 << cnt << "," << time_stamping << "," << robot_x << "," << robot_y << "," << robot_yaw << "," << beego_x << "," << beego_y << "," << beego_yaw 
                << ","<< estimate_odom_x << "," << estimate_odom_y << "," << estimate_theta << "," << likelihood_state << "," << max_likelyhood << "," << sum_likelyhood 
                << "," << i << ","<< particle_cloud.poses[i].position.x << "," << particle_cloud.poses[i].position.y << "," << yaw << "," << likelyhood_array[i] 
                << "," << v << "," << omega << "," << particle_cloud.poses[i].position.x - beego_x << "," << particle_cloud.poses[i].position.y - beego_y << "," << particle_i_yaw - beego_yaw 
                << "," << robot_x - beego_x << "," << robot_y - beego_y << "," << robot_yaw - beego_yaw << "," << estimate_odom_x - beego_x <<"," << estimate_odom_y - beego_y 
                << "," << estimate_theta - beego_yaw << std::endl;
        }
        
        ofs2 << cnt <<", " << time_stamping <<", " << beego_x <<", " << estimate_odom_x<< ", " << v << ", " << omega << std::endl;


        // //走行中だけリサンプリングを行う
        if (v > 0.04 || std::fabs(omega) > 0.002)
        {
            printf("cnt: %d, リサンプリング中\n",cnt);
            resampling();
        }
        else 
        // std::cout << "時間 "<< time_stamp << std::endl;
        {
            std::cout << "NOTリサンプリング" << std::endl;
        }


        if (likelihood_state == 1)
        {
            std::cout << "尤度更新中" << std::endl;
        }
        else
        {
            std::cout << "尤度更新しない" << std::endl;
        }   


        cout << "----------------------------------------------------------------------------"<<endl;

        self_localization_pub.publish(posi);
        particle_cloud_pub.publish(particle_cloud);

        // -----------------------------------transformaer

        tf::StampedTransform transform_esti_pose;
        transform_esti_pose.setOrigin(tf::Vector3(estimate_posi.pose.pose.position.x, estimate_posi.pose.pose.position.y, estimate_posi.pose.pose.position.z));
        transform_esti_pose.setRotation(tf::Quaternion(estimate_posi.pose   .pose.orientation.x,estimate_posi.pose.pose.orientation.y,estimate_posi.pose.pose.orientation.z,estimate_posi.pose.pose.orientation.w));
        // ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
        transform_esti_pose.stamp_ = ros::Time::now();
        transform_esti_pose.frame_id_ = "map";
        transform_esti_pose.child_frame_id_ = "dha_esti_pose";
        broadcaster.sendTransform(transform_esti_pose);

        geometry_msgs::PoseWithCovarianceStamped estimate_posi;
        estimate_posi.header.frame_id = "dha_esti_pose";
        esitimate_position_pub.publish(estimate_posi);
        max_estimate_position_pub.publish(max_estimate_posi);



        start = current; // 次のループのために現在時刻を更新
        likelyhood_array[maxIndex] = 0.0; //尤度の格納配列を0で初期化
        rate.sleep();
        cnt++;
        
        
        
    }
    ofs1.close();
    ofs2.close();

}
