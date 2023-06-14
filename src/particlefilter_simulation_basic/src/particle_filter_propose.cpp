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



sensor_msgs::PointCloud line_posi;
geometry_msgs::PoseArray particle_cloud; 
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseWithCovarianceStamped estimate_posi;

// 軌跡の保存用csvファイル
// std::ofstream ofs("2023-03-29_v0.5w1.0472_time_stamp_hyouka.csv");


// 初期速度指令値
double v = 0.0;
double omega = 0;
const double DT = 1.0 / 10.0;

double time_stamp = 0.0;
double amcl_pose_x = 0.0;
double amcl_pose_y = 0.0;
double robot_x = 0.0;
double robot_y = 0.0;





// 乱数生成エンジン
std::random_device seed;
std::mt19937 engine(seed());  

// パーティクルに付与するノイズ
double sigma_v_v = 0.03;  // 直進1mで生じる距離の標準偏差
double sigma_omega_v = 0.001; // 回転1radで生じる距離の誤差
double sigma_v_omega = 0.05;  //直進1mで生じる回転の誤差
double sigma_omega_omega = 0.1;   //回転1radで生じる回転の誤差
std::normal_distribution<> dist_v_v(0.0, sigma_v_v);
std::normal_distribution<> dist_omega_v(0.0, sigma_omega_v);
std::normal_distribution<> dist_v_omega(0.0, sigma_v_omega);
std::normal_distribution<> dist_omega_omega(0.0, sigma_omega_omega);


// パーティクル数
const int PARTICLE_NUM  = 500; 
double avg = 0.0, sig = 0.01; //平均0,標準偏差0.3(初期の散らばり具合)

std::vector<double> particle_value(PARTICLE_NUM); // particleの重み
    
// パーティクルの姿勢
double theta_pt[PARTICLE_NUM]= {0.0};


// 白線のポリゴン情報マップ(x_bottom, x_top, y_right, y_left)
const int POLYGON_NUM = 10; //ポリゴンの数l棟は8,sbは10
// sb棟において、エレベータ側の直線をスタート地点とした場合
double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545},{9.05, 9.125, 0.545, 1.935},{9.125, 10.205, 1.86, 1.935}, {10.13, 10.205,1.935, 3.01},{11.28, 11.355, 1.76, 3.01},{11.355, 13.5, 1.76, 1.835},{11.41,13.5, 0.75, 0.825},{11.41,11.485, 0.75, -5.0},{10.02,10.095, -0.545, -5.0},{0,10.095, -0.47, -0.545}};

// joy
double x_joy = 0.0;
double w_joy = 0.0;


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

void amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    amcl_pose_x = msg->pose.pose.position.x;
    amcl_pose_y = msg->pose.pose.position.y;    
}


// cmd_velとodomの値をcallback
void velCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    v = msg->twist.twist.linear.x;
    omega = msg->twist.twist.angular.z;
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

// cbPoint
void line_point_cb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    line_posi.points = msg->points;
}

// joyのcmd_velをcallback
void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    x_joy = joy_msg.axes[1];
    w_joy = joy_msg.axes[3];

}

// パーティクル数で割って重みの正規化（重みの合計が1になるようにしてる）
void likelihood_value_nomalization()
{
    double sum = 0;
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        sum += particle_value[i];
    }
    // std::cout << "パーティクルの重みの和" << sum << std::endl;
    printf("パーティクルの重みの和 %.32f \n",sum);
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        
        if (particle_value[i] != 0)
        {
            particle_value[i] = double(particle_value[i]) / sum; 
        }
        if (double(sum) == 0.0)//デバッグ用
        {
            printf("sumがゼロになった\n");
        }

    }
}


// リサンプリングの処理
void resampling(double ws) // 重みの和を受け取る
{
    std::cout << "a" << std::endl;

    // double rand_width = 1.0 / double(PARTICLE_NUM); // 乱数幅 : 1じゃなくて、806重みの和
    double rand_width = ws / double(PARTICLE_NUM);
    std::uniform_real_distribution<float> distr(0, rand_width); // 0~rand_widthの範囲で当確率で発生する
    double rand = distr(engine); // 積み上げ乱数
    double w_sum = particle_value[0]; //積み上げ正規化重み
    int n_before = 0; // リサンプリング前の重みのインデックス
    int n_after = 0; // リサンプリング後の重みのインデックス

    
    geometry_msgs::PoseArray particle_new; // リサンプリング後の粒子
    std::vector<double> particle_value_new(PARTICLE_NUM); //リサンプリング後の粒子の重み
    particle_new.poses.resize(PARTICLE_NUM); //

    // 要素のコピー
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        particle_new.poses[i] = particle_cloud.poses[i];
        particle_value_new[i] = particle_value[i];
        
    }
    // 系統リサンプリングの処理開始
    std::cout << "b" << std::endl;


    while (n_after < PARTICLE_NUM)
    {
        
        std::cout << "処理のまえ　" << "積み上げ乱数 " << rand << " " << "w_sum " << w_sum << " " << "一つ一つの重み " << particle_value[n_before] << " " <<"n_after " << n_after << " " << "n_before " << n_before << std::endl;
        // if (n_before > PARTICLE_NUM -5)
        // {
        //     std::cout << "超えそう" << n_before << "/" << PARTICLE_NUM <<  std::endl;
        // }
        // if (n_before > PARTICLE_NUM)
        // {
        //     std::cout << "n_beforeがパーティクル数を超えた" << n_before << "/" << PARTICLE_NUM <<  std::endl;
        // }
        // if (n_after > PARTICLE_NUM)
        // {
        //     std::cout << "n_afterがパーティクル数を超えた" << n_after <<  std::endl;
        // }
        // std::cout </< "n_before " << n_before << "n_after " << n_after << std::endl; 


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
        // std::cout << "処理のあと　" << "積み上げ乱数 " << rand << " " << "w_sum " << w_sum << " " << "一つ一つの重み " << particle_value[n_before] << " " <<"n_after " << n_after << " " << "n_before " << n_before << std::endl;

    }

    std::cout << "c" << std::endl;

    // 元のパーティクルに代入
    for (int i=0; i < PARTICLE_NUM; i++)
    {
        // リサンプリング後のパーティクルの更新
        particle_cloud.poses[i] = particle_new.poses[i];
        particle_value[i] = ws / double(PARTICLE_NUM); // 重みの初期化
        // printf("%")
        std::cout << "リサンプリング後の代入した重み　" << particle_value[i] << " ws " << ws << std::endl;

    }
    

}



// メイン
int main(int argc, char **argv)
{

    ros::init(argc, argv, "particle_node");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher self_localization_pub = nh.advertise<nav_msgs::Odometry>("self_position", 10); // robotにモータに指令値を送るpublisherの宣言。
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5); // リサンプリング後のパーティクル
    ros::Publisher esitimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate_position",5);// posewithcoverianceで推定値をpublish
    
    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, velCallback);
    ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);
    ros::Subscriber joy_vel_sub = nh.subscribe("/joy",1, joy_callback);
    ros::Subscriber amcl_pose_sub = nh.subscribe("/amcl_pose", 10, amcl_poseCallback);

    
    nav_msgs::Odometry posi;
    

    posi.header.frame_id = "map";
    estimate_posi.header.frame_id = "map";
    particle_cloud.header.frame_id = "map";
    particle_cloud.header.stamp = ros::Time::now();
    particle_cloud.poses.resize(PARTICLE_NUM);


    
    // 初期のparticleの位置と姿勢
    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        //ガウス分布（平均、標準偏差）
        std::normal_distribution<> dist_coodinate(avg, sig); //sigmaをいじれば初期のパーティクルのばらつきが決まる
        std::normal_distribution<> dist_theta(0.0, 0.01); //姿勢の散らばり

        particle_cloud.poses[i].position.x = dist_coodinate(engine);
        particle_cloud.poses[i].position.y = dist_coodinate(engine);
        particle_cloud.poses[i].position.z = 0.0;
        particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0, 0, dist_theta(engine)); 
        particle_value[i] = 1.0 / double(PARTICLE_NUM); //最初の重みは均一

    }



    ros::Rate rate(10.0);
    int cnt = 0;
 
    while (ros::ok())
    {
        time_stamp += DT; // dtで時間を積算


        ros::spinOnce();

        // パーティクルの位置に状態遷移モデルの適用
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            // ノイズをのせたvとomegaを生成
            double v_noise,omega_noise;
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            v_noise = v + dist_v_v(engine) * sqrt(abs(v)/DT) + dist_v_omega(engine) * sqrt(abs(omega)/DT);
            omega_noise = omega + dist_omega_v(engine) * sqrt(abs(v)/DT) + dist_omega_omega(engine) * sqrt(abs(omega)/DT);

            particle_cloud.poses[i].position.x = particle_cloud.poses[i].position.x + v_noise * cos(yaw)*DT;
            particle_cloud.poses[i].position.y = particle_cloud.poses[i].position.y + v_noise * sin(yaw)*DT;
            particle_cloud.poses[i].position.z = 0.0; 
            particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0,0, yaw + omega_noise * DT);
            // std::cout << "状態遷移モデル内でのprint" << particle_cloud.poses[i].position.x << std::endl;
        
        }

        if (line_posi.points.size() > 0) // このif文を外すと動かない
        {
            // 尤度計算
            double line_vertical_small = 0;
            double line_vertical_big   = 0;
            double line_side_small     = 0;
            double line_side_big       = 0;
            double likelihood_value    = 0;
            
            
            for (int i = 0; i < PARTICLE_NUM; i++) //パーティクル一個一個について計算していく
            {
                double roll, pitch, yaw;
                geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
                int match_count = 0; //尤度のマッチ数
                for (int j = 0; j < line_posi.points.size(); j++)//検出した白線の座標配列
                {
                    // particle一つ一つについて絶対座標に変換
                    double line_glx = line_posi.points[j].x * cosf(yaw) - line_posi.points[j].y * sinf(yaw) +  particle_cloud.poses[i].position.x;
                    double line_gly = line_posi.points[j].x * sinf(yaw) + line_posi.points[j].y * cosf(yaw) +  particle_cloud.poses[i].position.y;
                    
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
                        }
                    }
                }

                // likelihood_value = double(match_count) / double(line_posi.points.size());
                
                // likelihood_value = match_count;
                // particle_value[i] *= likelihood_value; //前回の重みｘ尤度＝今回の重み
                particle_value[i] = match_count;


                // 重みの和を計算
                double ws = 0;
                for(int i = 0; i < particle_value.size(); i++)
                {   
                    ws += particle_value[i];     
                }
                
                printf("尤度計算内での重みの和wsを算出%.32f\n", ws);
                printf("match_countを算出%d\n", match_count);

                if (match_count != 0)
                {
                    resampling(ws); // 白線点群が一つもマッチしなかったときはリサンプリングをスキップ    
                }
                else 
                {
                    printf("match_countはゼロ\n");
                }
                
            }


            // 重みの正規化 : particle_valueの値を正規化
            // likelihood_value_nomalization();
  
            //走行中だけリサンプリングを行う
            // std::cout << "v: " << v << " " << "w: " << omega << std::endl; 
            // 常にリサンプリングをするとこの問題が起きるかを調査する
            // resampling(cnt);
            // if (v > 0.002 || std::fabs(omega) > 0.002)
            // {
            //     printf("cnt: %d, リサンプリング中\n",cnt);
            //     resampling(cnt);
            // }
            // else 
            // {
            //     printf("cnt: %d, なにもしない\n",cnt);
            // }
            


            // // 推定結果の自己位置
            // double estimate_odom_x = 0.0;
            // double estimate_odom_y = 0.0;
            // double estimate_theta  = 0.0;

            // // 自己位置を出す処理：自己位置 +＝ 重み[i] * (x,y,theta)[i]
            // for (int i = 0; i < PARTICLE_NUM; i++)
            // {
            //     double roll, pitch, yaw;
            //     geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            //     estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            //     estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            //     estimate_theta  += particle_value[i] * yaw;

            // }

            // estimate_posi.pose.pose.position.x = estimate_odom_x;
            // estimate_posi.pose.pose.position.y = estimate_odom_y;
            // estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, estimate_theta);
            // std::cout << "推定x " << estimate_posi.pose.pose.position.x << " 推定Y " << estimate_posi.pose.pose.position.y << std::endl;

            // 推定した結果をcsvに出力
            // ofs << time_stamp << ", " << robot_x << ", " << robot_y << ", " << amcl_pose_x << ", " << amcl_pose_y << ", " << estimate_odom_x << ", " << estimate_odom_y << std::endl;

            cnt++;
            

            self_localization_pub.publish(posi);
            particle_cloud_pub.publish(particle_cloud);
            // esitimate_position_pub.publish(estimate_posi);

            
        }

        rate.sleep();
        
    }

}
