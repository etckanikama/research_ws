#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <time.h>
using namespace std;

int p_n = 1000; //particleの数
ros::Subscriber sub_odom_;
ros::Subscriber sub_scan_;
nav_msgs::Odometry odom_;
sensor_msgs::LaserScan latest_scan_;
double scanLC[1081][2];
int comb[9][511];
vector<pair<double, double>> mapGL;
geometry_msgs::PoseArray particle_cloud;
vector<double> particle_value(p_n); // particleの重み
vector<int> particle_id(p_n);       // 参照するパターンの番号
vector<int> update_pos(p_n);

int time_span = 5;
double odom_diff_x, odom_diff_y, odom_pre_x, odom_pre_y;
double velocity = 0.2;  //速度[m/s]
double omega = 0;       // 角速度[rad/s]
double sigma_a = 0.03;  // 直進1mで生じる距離の標準偏差
double sigma_b = 0.001; // 回転1radで生じる距離の誤差
double sigma_c = 0.05;  //直進1mで生じる回転の誤差
double sigma_d = 0.1;   //回転1radで生じる回転の誤差
random_device seed_gen;
default_random_engine engine(seed_gen());
normal_distribution<> dist_a(0.0, sigma_a);
normal_distribution<> dist_b(0.0, sigma_b);
normal_distribution<> dist_c(0.0, sigma_c);
normal_distribution<> dist_d(0.0, sigma_d);

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;
}
void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    latest_scan_ = *msg;
    double theta;

    for (int i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] > 0.3 && msg->ranges[i] < 30)
        {
            theta = (msg->angle_max - msg->angle_min) / msg->ranges.size() * i + msg->angle_min;
            scanLC[i][0] = msg->ranges[i] * cos(theta) - 0.012; //センサ座標からロボット座標に変換
            scanLC[i][1] = msg->ranges[i] * sin(theta);
        }
        else
        {
            scanLC[i][0] = 0;
            scanLC[i][1] = 0;
        }
    }
}

tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw)
{
    return tf::createQuaternionFromRPY(roll, pitch, yaw);
}
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}
void tf_quat_to_rpy(double &roll, double &pitch, double &yaw, tf::Quaternion quat)
{
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}

void init()
{
    //MAPの読み込み
    FILE *MAP_data;
    // char MAP_data_file[] = "/home/miura-t/catkin_ws/data/map/sb_map.txt";
    char MAP_data_file[] = "/home/miura-t/catkin_ws/data/map/sb_map_real.txt";
    if ((MAP_data = fopen(MAP_data_file, "r")) == NULL)
    {
        fprintf(stderr, "%s\n", "error: can't read file.");
        // return EXIT_FAILURE;
    }
    double x, y;
    while (fscanf(MAP_data, "%lf %lf", &x, &y) != EOF)
    {
        pair<double, double> p;
        p.first = x;
        p.second = y;
        mapGL.push_back(p);
    }
    fclose(MAP_data);

    // スタート時に分散させる処理 必要
    for (int i = 0; i < p_n; i++)
    {
        normal_distribution<> dist_x(0.0, 0.03);
        normal_distribution<> dist_theta(0.0, 0.09);
        particle_cloud.poses[i].position.x = dist_x(engine);
        particle_cloud.poses[i].position.y = dist_x(engine);
        particle_cloud.poses[i].position.z = 0.3;

        particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0, 0, dist_theta(engine));
        particle_value[i] = 1.0 / p_n; //最初の重みは均一
        particle_id[i] = rand() % 511;
    }
    // 初期位置のファイル出力
    ofstream writing_file0;
    string filename0 = "particle0.txt";
    writing_file0.open(filename0, std::ios::out);
    for (int i = 0; i < p_n; i++)
    {
        double roll, pitch, yow;
        geometry_quat_to_rpy(roll, pitch, yow, particle_cloud.poses[i].orientation);
        writing_file0 << particle_cloud.poses[i].position.x << " " << particle_cloud.poses[i].position.y << " " << yow << " " << particle_value[i] << endl;
    }
}

void getValue();
void normalization();
void resampling();

bool RESAMPLE = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_simulation"); //第３引数はノードの名前
    ros::NodeHandle n;
    ros::Publisher pub_cloud = n.advertise<geometry_msgs::PoseArray>("particle_cloud", 5);
    particle_cloud.poses.resize(p_n);
    particle_cloud.header.frame_id = std::string("beego/odom"); //gazeboのframe_idがbeego/odomになっているから？？？　よくわからない
    particle_cloud.header.stamp = ros::Time::now();

    init();
    ros::spinOnce();

    ros::Rate rate(10);

    int N = 5;
    // 何回ファイルを読み込むか
    clock_t start = clock();
    for (int CNT = 1; CNT <= N; CNT++)
    {
        ros::spinOnce();
        // ファイルから点群を読み込む処理
        string file_name = "./point" + to_string(CNT) + ".txt";
        ifstream ifs(file_name);
        string str;
        int j = 0;
        while (getline(ifs, str))
        {
            istringstream iss(str);
            string a, b;
            iss >> a >> b;
            scanLC[j][0] = stod(a);
            scanLC[j][1] = stod(b);
            j++;
        }

        string odom_file = "./odom" + to_string(CNT) + ".txt";
        ifstream ifs2(odom_file);
        double odom_x, odom_y, odom_rad;
        ifs2 >> odom_x >> odom_y >> odom_rad;
        double odom_dis = sqrt(pow(odom_x, 2) + pow(odom_y, 2));

        //状態遷移モデルの実装
        for (int i = 0; i < p_n; i++)
        {
            double v_noise, omega_noise;
            v_noise = velocity + dist_a(engine) + dist_b(engine) * sqrt(abs(omega) / time_span);
            omega_noise = omega + dist_c(engine) * sqrt(abs(velocity) / time_span) + dist_d(engine) * sqrt(abs(omega) / time_span);

            double roll, pitch, yow;
            geometry_quat_to_rpy(roll, pitch, yow, particle_cloud.poses[i].orientation);

            double local_x = odom_x + dist_a(engine) * odom_x + dist_b(engine) * odom_rad;
            double local_y = odom_y + dist_a(engine) * odom_y + dist_b(engine) * odom_rad;

            particle_cloud.poses[i].position.x += local_x * cos(yow) - local_y * sin(yow);
            particle_cloud.poses[i].position.y += local_x * sin(yow) + local_y * cos(yow);

            yow += odom_rad + dist_c(engine) * odom_dis + dist_d(engine) * odom_rad; // radだから加算するだけ

            particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0, 0, yow); // radをquatに戻す
        }

        RESAMPLE = false; // 全ての尤度が0のときに対応する。 一時的な対応
        getValue();       //尤度の計算

        normalization(); // 重みの正規化

        // パーティクルの位置の出力(リサンプリング前)
        ofstream writing_file2;
        string filename2 = "particle" + to_string(CNT) + "_before.txt";
        writing_file2.open(filename2, std::ios::out);
        for (int i = 0; i < p_n; i++)
        {
            double roll, pitch, yow;
            geometry_quat_to_rpy(roll, pitch, yow, particle_cloud.poses[i].orientation);
            writing_file2 << particle_cloud.poses[i].position.x << " " << particle_cloud.poses[i].position.y << " " << yow << " " << particle_value[i] << endl;
            cout << particle_value[i] << endl;
        }

        resampling(); //リサンプリング

        // パーティクルの位置の出力(リサンプリング後)
        ofstream writing_file3;
        string filename3 = "particle" + to_string(CNT) + "_after.txt";
        writing_file3.open(filename3, std::ios::out);
        for (int i = 0; i < p_n; i++)
        {
            double roll, pitch, yow;
            geometry_quat_to_rpy(roll, pitch, yow, particle_cloud.poses[i].orientation);
            writing_file3 << particle_cloud.poses[i].position.x << " " << particle_cloud.poses[i].position.y << " " << yow << " " << particle_value[i] << endl;
        }

        pub_cloud.publish(particle_cloud);
        cout << "-----------------" << endl;
    }
    clock_t end = clock(); // 終了時間
    std::cout << "time = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
}

// パーティクルの尤度を計算する
void getValue()
{
    for (int i = 0; i < p_n; i++)
    {
        int angle = 30;
        double value = 0; // 誤差の和
        int cnt = 0;      //使った点群数
        double roll, pitch, yow;
        geometry_quat_to_rpy(roll, pitch, yow, particle_cloud.poses[i].orientation);
        //観測点それぞれに対する処理
        for (int j = 0; j < 1081; j++)
        {
            double x = scanLC[j][0];
            double y = scanLC[j][1];
            if (x != 0 && y != 0)
            {
                // globalに変換
                double x_gl = x * cos(yow) - y * sin(yow);
                double y_gl = x * sin(yow) + y * cos(yow);
                x_gl += particle_cloud.poses[i].position.x;
                y_gl += particle_cloud.poses[i].position.y;

                double dis;
                double min_dis = 10000;
                //まずは対応点を見つける
                for (int k = 0; k < mapGL.size(); k++)
                {
                    dis = pow((x_gl - mapGL[k].first), 2) + pow((y_gl - mapGL[k].second), 2);
                    min_dis = min(min_dis, sqrt(dis));
                }
                cnt++;
                value += min_dis;
            }
        }
        value = value / cnt;
        // 尤度の変換
        // double border = 0.5;
        // if (value >= border)
        // {
        //     value = 0;
        //     particle_value[i] = 0;
        // }
        // else
        // {
        //     value = 1 - (value / border);
        //     particle_value[i] = value;
        //     RESAMPLE = true;
        // }

        particle_value[i] = exp(-4.605 * value);
    }
}

void normalization()
{
    double sum = 0;
    for (int i = 0; i < p_n; i++)
    {
        sum += particle_value[i];
    }
    for (int i = 0; i < p_n; i++)
    {
        if (particle_value[i] != 0)
        {
            particle_value[i] = particle_value[i] / sum;
        }
    }
}
void resampling()
{
    // 要素のコピー
    geometry_msgs::PoseArray particle_tmp;
    vector<double> particle_value_tmp(p_n);
    particle_tmp.poses.resize(p_n);
    for (int i = 0; i < p_n; i++)
    {
        particle_tmp.poses[i].position.x = particle_cloud.poses[i].position.x;
        particle_tmp.poses[i].position.y = particle_cloud.poses[i].position.y;
        particle_tmp.poses[i].orientation = particle_cloud.poses[i].orientation;
        particle_value_tmp[i] = particle_value[i];
    }

    random_device rd;
    default_random_engine eng(rd());
    uniform_real_distribution<float> distr(0, 1);
    for (int i = 0; i < p_n; i++)
    {
        double rand_val = distr(eng);
        // cout << rand_val << endl;

        // rand_valを指す要素はcnt
        double sum = 0;
        int cnt = 0;
        while (sum <= rand_val)
        {
            sum += particle_value_tmp[cnt];
            cnt++;
        }
        cnt--;
        particle_cloud.poses[i].position.x = particle_tmp.poses[cnt].position.x;
        particle_cloud.poses[i].position.y = particle_tmp.poses[cnt].position.y;
        particle_cloud.poses[i].orientation = particle_tmp.poses[cnt].orientation;
        particle_value[i] = 1.0 / p_n;
    }
}
