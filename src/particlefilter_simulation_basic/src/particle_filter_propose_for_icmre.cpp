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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <tf/transform_broadcaster.h>

// icmre用の尤度計算改良版

sensor_msgs::PointCloud line_posi;
geometry_msgs::PoseArray particle_cloud; 
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseWithCovarianceStamped estimate_posi;

// 軌跡の保存用csvファイル
std::ofstream ofs("/home/hirayama-d/research_ws/src/particlefilter_simulation_basic/csv_ICMRE/2023-09-28_yuudokansuu_kairyou_v4.csv");


// 初期速度指令値
double v = 0.0;
double omega = 0;
const double dt = 1.0 / 10.0;

double time_stamp = 0.0;
double amcl_pose_x = 0.0;
double amcl_pose_y = 0.0;
double robot_x = 0.0;
double robot_y = 0.0;





// 乱数生成エンジン
std::random_device seed;
std::mt19937 engine(seed());  

// パーティクルに付与するノイズ
// double sigma_v_v = 0.06;  // 直進1mで生じる距離の標準偏差
// double sigma_omega_v = 0.002; // 回転1radで生じる距離の誤差
// double sigma_v_omega = 0.1;  //直進1mで生じる回転の誤差
// double sigma_omega_omega = 0.2;   //回転1radで生じる回転の誤差

double sigma_v_v = 0.03;  // 直進1mで生じる距離の標準偏差
double sigma_omega_v = 0.001; // 回転1radで生じる距離の誤差
double sigma_v_omega = 0.05;  //直進1mで生じる回転の誤差
double sigma_omega_omega = 0.1;   //回転1radで生じる回転の誤差
std::normal_distribution<> dist_v_v(0.0, sigma_v_v);
std::normal_distribution<> dist_omega_v(0.0, sigma_omega_v);
std::normal_distribution<> dist_v_omega(0.0, sigma_v_omega);
std::normal_distribution<> dist_omega_omega(0.0, sigma_omega_omega);



const int PARTICLE_NUM  = 500; // パーティクル数
double coeff = 0.1; //尤度係数
double Mth = 300; //最低限含まれてほしい点群数のしきい値
double sigma_px_th =0.15;  //x方向の標準偏差のしきい値:白線の幅(0.075)*2=0.15
double sigma_py_th =0.15; //y方向の標準偏差のしきい値:白線の幅(0.075)*2=0.15


std::vector<double> particle_value(PARTICLE_NUM); // particleの重み
double theta_pt[PARTICLE_NUM]= {0.0};// パーティクルの姿勢


// 白線のポリゴン情報マップ(x_bottom, x_top, y_right, y_left)
const int POLYGON_NUM = 10; //ポリゴンの数l棟は8,sbは10
// sb棟において、エレベータ側の直線をスタート地点とした場合
// double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545},{9.05, 9.125, 0.545, 1.935},{9.125, 10.205, 1.86, 1.935}, {10.13, 10.205,1.935, 3.01},{11.28, 11.355, 1.76, 3.01},{11.355, 13.5, 1.76, 1.835},{11.41,13.5, 0.75, 0.825},{11.41,11.485, 0.75, -5.0},{10.02,10.095, -0.545, -5.0},{0,10.095, -0.47, -0.545}};
double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545},{9.05, 9.125, 0.545, 1.935},{9.125, 10.205, 1.86, 1.935}, {10.13, 10.205,1.935, 3.01},{11.28, 11.355, 1.76, 3.01},{11.355, 13.5, 1.76, 1.835},{11.41,13.5, 0.75, 0.825},{11.41,11.485, 0.75, -5.0},{10.02,10.095, -0.545, -5.0},{0,10.095, -0.47, -0.545}};



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
void resampling(int cnt)
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



// メイン
int main(int argc, char **argv)
{

    ros::init(argc, argv, "particle_node_kairyou");
    ros::NodeHandle nh;

    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;// rosbagの初期値にしたがって変更
    std::cout << "icrme"<< std::endl;

    // Publisher
    ros::Publisher self_localization_pub = nh.advertise<nav_msgs::Odometry>("self_position", 10); // robotにモータに指令値を送るpublisherの宣言。
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5); // リサンプリング後のパーティクル
    ros::Publisher esitimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("kairyou_estimate_position",5);// posiewithcoverianceで推定値をpublish
    
    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, velCallback);
    ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);
    ros::Subscriber amcl_pose_sub = nh.subscribe("/amcl_pose", 10, amcl_poseCallback);

    
    nav_msgs::Odometry posi;
    tf::TransformBroadcaster broadcaster;
    

    posi.header.frame_id = "map";
    // estimate_posi.header.frame_id = "map";
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

    
    while (ros::ok())
    {
        time_stamp += dt; // dtで時間を積算

        ros::spinOnce();

        // パーティクルの位置に状態遷移モデルの適用
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            // ノイズをのせたvとomegaを生成
            double v_noise,omega_noise;
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            v_noise = v + dist_v_v(engine) * sqrt(abs(v)/dt) + dist_v_omega(engine) * sqrt(abs(omega)/dt);
            omega_noise = omega + dist_omega_v(engine) * sqrt(abs(v)/dt) + dist_omega_omega(engine) * sqrt(abs(omega)/dt);

            particle_cloud.poses[i].position.x = particle_cloud.poses[i].position.x + v_noise * cos(yaw)*dt;
            particle_cloud.poses[i].position.y = particle_cloud.poses[i].position.y + v_noise * sin(yaw)*dt;
            particle_cloud.poses[i].position.z = 0.0; 
            particle_cloud.poses[i].orientation = rpy_to_geometry_quat(0,0, yaw + omega_noise * dt);
        
        }



        if (line_posi.points.size() > Mth) //最低限Mth以上含まれている時だけ尤度計算を行う
        {
            // -------------標準偏差の計算----------------------
            // sumの作成
            double sum_px = 0.0, sum_py = 0.0;
            for (int j = 0; j < line_posi.points.size(); j++)
            {
                sum_px += line_posi.points[j].x;
                sum_py += line_posi.points[j].y;
            }
            // meanの作成
            double mean_px = sum_px / line_posi.points.size();
            double mean_py = sum_py / line_posi.points.size();
            //偏差の二乗
            double sumOfSquaredDeviation_x = 0.0;
            double sumOfSquaredDeviation_y = 0.0;
            for (int j = 0; j < line_posi.points.size(); j++)
            {
                double deviation_px = line_posi.points[j].x - mean_px;
                double deviation_py = line_posi.points[j].y - mean_py;
                sumOfSquaredDeviation_x += deviation_px * deviation_px;
                sumOfSquaredDeviation_y += deviation_py * deviation_py;
            }

            //標準偏差を算出
            double sigma_px = std::sqrt(sumOfSquaredDeviation_x / (line_posi.points.size() - 1));
            double sigma_py = std::sqrt(sumOfSquaredDeviation_y / (line_posi.points.size() - 1));
            // -----------------------------------------------------
            std::cout << line_posi.points.size() << std::endl;
            std::cout << "x方向の標準偏差 " << sigma_px << " " << "y方向の標準偏差" << sigma_py << std::endl;

            if ((sigma_px > sigma_px_th) && (sigma_py > sigma_py_th)) //この条件を突破したら尤度計算が始まる
            {
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
                    
                    for (int j = 0; j < line_posi.points.size(); j++)//尤度計算の中のループ
                    {
                        // particle一つ一つを使って白線点群を絶対座標に変換
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
                                break;
                            }
                        }
                    }

                    likelihood_value = (1 - coeff)*(double(match_count) / double(line_posi.points.size())) + coeff;
                    // // likelihood_value = 1 / (1 + PARTICLE_NUM * coeff) * (double(match_count) / double(line_posi.points.size())) + coeff;
                    particle_value[i] *= likelihood_value; //前回の重みｘ尤度＝今回の重み


                }
                std::cout << "尤度 " << likelihood_value << std::endl;
            }



        }



       


            

        // 重みの正規化 : particle_valueの値を正規化
        likelihood_value_nomalization();
        // 推定結果の自己位置
        double estimate_odom_x = 0.0;
        double estimate_odom_y = 0.0;
        double estimate_theta  = 0.0;

        // 重みづけ平均：自己位置を出す処理：自己位置 +＝ 重み[i] * (x,y,theta)[i]
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            double roll, pitch, yaw;
            geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            estimate_theta  += particle_value[i] * yaw;

        }

        std::cout << "推定値 x " << estimate_odom_x << " "<< "オドメトリ x " << robot_x <<std::endl;
        // ______________尤度の最大値を推定値とする__________________________________
        // double max_value = particle_value[0];
        // int max_value_index = 0;
        // for (int i = 0; i < PARTICLE_NUM; i++)
        // {

        //     if (max_value < particle_value[i])
        //     {
        //         max_value = particle_value[i];
        //         max_value_index = i;
        //     }

        //     // estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
        //     // estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
        //     // estimate_theta  += particle_value[i] * yaw;
        // }



        // double roll, pitch, yaw;
        // geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[max_value_index].orientation); //yaw角に変換
        // estimate_odom_x =   particle_cloud.poses[max_value_index].position.x;
        // estimate_odom_y = particle_cloud.poses[max_value_index].position.y;
        // estimate_theta  = yaw;
        // ------------------------------------------------------------

        estimate_posi.pose.pose.position.x = estimate_odom_x;
        estimate_posi.pose.pose.position.y = estimate_odom_y;
        estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, estimate_theta);


        // //走行中だけリサンプリングを行う
        if (v > 0.04 || std::fabs(omega) > 0.02)
        {
            printf("cnt: %d, リサンプリング中\n",cnt);
            resampling(cnt);
        }
        else 
        {
            std::cout << "NOTリサンプリング" << std::endl;
        }

        std::cout << "時間 "<< time_stamp << std::endl;



        // // 推定した結果をcsvに出力
        ofs << time_stamp << ", " << robot_x << ", " << robot_y << ", " << amcl_pose_x << ", " << amcl_pose_y << ", " << estimate_odom_x << ", " << estimate_odom_y << std::endl;

        // cnt++;
        

        self_localization_pub.publish(posi);
        particle_cloud_pub.publish(particle_cloud);
        // esitimate_position_pub.publish(estimate_posi);



        tf::StampedTransform transform_esti_pose;
        transform_esti_pose.setOrigin(tf::Vector3(estimate_posi.pose.pose.position.x, estimate_posi.pose.pose.position.y, estimate_posi.pose.pose.position.z));
        transform_esti_pose.setRotation(tf::Quaternion(estimate_posi.pose.pose.orientation.x,estimate_posi.pose.pose.orientation.y,estimate_posi.pose.pose.orientation.z,estimate_posi.pose.pose.orientation.w));
        // ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
        transform_esti_pose.stamp_ = ros::Time::now();
        transform_esti_pose.frame_id_ = "map";
        transform_esti_pose.child_frame_id_ = "dha_esti_pose";
        broadcaster.sendTransform(transform_esti_pose);

        geometry_msgs::PoseWithCovarianceStamped estimate_posi;
        estimate_posi.header.frame_id = "dha_esti_pose";
        esitimate_position_pub.publish(estimate_posi);

            
        

        rate.sleep();
        
    }

}
