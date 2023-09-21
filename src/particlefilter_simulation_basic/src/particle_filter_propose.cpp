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
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float32MultiArray.h>


sensor_msgs::PointCloud line_posi;
geometry_msgs::PoseArray particle_cloud; 
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseWithCovarianceStamped estimate_posi;
// pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
pcl::PointCloud<pcl::PointXYZ> downsampled_cloud, global_white_lane_points;
sensor_msgs::PointCloud2 global_cloud_points;
std_msgs::Float32MultiArray weights;
// 軌跡の保存用csvファイル
// std::ofstream ofs("メディアンなし&ボクセルなし2.0以下.csv");


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


// パーティクルに付与するノイズ:元のやつ
double sigma_v_v = 0.03;  // 直進1mで生じる距離の標準偏差0.03
double sigma_omega_v = 0.001; // 回転1radで生じる距離の誤差0.001
double sigma_v_omega = 0.5;  //直進1mで生じる回転の誤差0.05
double sigma_omega_omega = 0.5;   //回転1radで生じる回転の誤差0.1
std::normal_distribution<> dist_v_v(0.0, sigma_v_v);
std::normal_distribution<> dist_omega_v(0.0, sigma_omega_v);
std::normal_distribution<> dist_v_omega(0.0, sigma_v_omega);
std::normal_distribution<> dist_omega_omega(0.0, sigma_omega_omega);


// パーティクル数
const int PARTICLE_NUM  = 500; 
double likely_coeff = 0.01;

std::vector<double> particle_value(PARTICLE_NUM); // particleの重み
    
// パーティクルの姿勢
double theta_pt[PARTICLE_NUM]= {0.0};


// 白線のポリゴン情報マップ(x_bottom, x_top, y_right, y_left)
const int POLYGON_NUM = 10; //ポリゴンの数l棟は8,sbは10
// // sb棟において、エレベータ側の直線をスタート地点とした場合：単位はメートル
double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545},{9.05, 9.125, 0.545, 1.935},{9.125, 10.205, 1.86, 1.935}, {10.13, 10.205,1.935, 3.01},{11.28, 11.355, 1.76, 3.01},{11.355, 13.5, 1.76, 1.835},{11.41,13.5, 0.75, 0.825},{11.41,11.485, 0.75, -5.0},{10.02,10.095, -0.545, -5.0},{0,10.095, -0.47, -0.545}};
// const int POLYGON_NUM = 25; // sbのポリゴンの数は駐車場加えて24(x_min,x_max,y_min,y_max)
// double WHITE_POLYGON_MAP[POLYGON_NUM][4] = {{0.0, 9.125, 0.47, 0.545}, {9.05, 9.125, 0.545, 1.935}, {9.125, 10.205, 1.86, 1.935}, {10.13, 10.205, 1.935, 3.01}, {11.28, 11.355, 1.76, 3.01}, {11.355, 14.0, 1.76, 1.835}, {11.41, 14.0, 0.75, 0.825}, {11.41, 11.485, 0.75, -5.0}, {10.02, 10.095, -0.545, -5.0}, {0, 10.095, -0.47, -0.545},
//                                             {0.92, 0.95, -1.465, -3.61},{0.95, 1.345,-1.465, -1.540},{1.345, 1.420, -1.465, -3.61},{2.25, 2.325, -1.465, -3.61},{2.325, 3.26,-1.465, -1.540},{3.26, 3.335, -1.465, -3.61},{4.15, 4.225, -1.465, -3.61},{4.225, 5.175,-1.465, -1.540},{5.175, 5.250, -1.465, -3.61},{6.050, 6.125, -1.465, -3.61},
//                                             {6.125, 7.085,-1.465, -1.540},{7.08, 7.16, -1.465, -3.61},{7.96, 8.035, -1.465, -3.61},{8.055, 9.305,-1.465, -1.540},{9.30, 9.38, -1.465, -3.61}};

int flg = 0;



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

// AMCLの推定値をサブスクライブする
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



// ダウンサンプリングした後の点群を読み込む
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
    if (double(sum) == 0.0)
    {
        printf("sumがゼロになった\n");
    }

    for (int i = 0; i < PARTICLE_NUM; i++)
    {
        
        if (particle_value[i] != 0)
        {
            particle_value[i] = double(particle_value[i]) / sum; 
        }

    }
}


// リサンプリングの処理
void resampling(int cnt)
{


    double rand_width = 1.0 / double(PARTICLE_NUM); // 乱数幅 : 1じゃなくて、806重みの和
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

        // std::cout << "処理のあと　" << "積み上げ乱数 " << rand << " " << "w_sum " << w_sum << " " << "一つ一つの重み " << particle_value[n_before] << " " <<"n_after " << n_after << " " << "n_before " << n_before << std::endl;

    }

    // 元のパーティクルに代入
    for (int i=0; i < PARTICLE_NUM; i++)
    {
        // リサンプリング後のパーティクルの更新
        particle_cloud.poses[i] = particle_new.poses[i];
        particle_value[i] = 1.0 / double(PARTICLE_NUM); // 重みの初期化

    }
    

}






// メイン
int main(int argc, char **argv)
{

    ros::init(argc, argv, "particle_node");
    ros::NodeHandle nh;

    
    //kiiro_01,kiiro_02, ao_01, ao_02
    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;// rosbagの初期値にしたがって変更


    // Publisher
    ros::Publisher self_localization_pub = nh.advertise<nav_msgs::Odometry>("self_position", 10); // robotにモータに指令値を送るpublisherの宣言。
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5); // リサンプリング後のパーティクル
    ros::Publisher esitimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate_position",5);// posewithcoverianceで推定値をpublish
    // ros::Publisher weights_pub = nh.advertise<std_msgs::Float32MultiArray>("/converted_weights", 10);

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom",10, velCallback);
    // ros::Subscriber line_points_sub = nh.subscribe("/front_camera/line_points", 1, line_point_cb);
    // ros::Subscriber joy_vel_sub = nh.subscribe("/joy",1, joy_callback);
    ros::Subscriber amcl_pose_sub = nh.subscribe("/amcl_pose", 10, amcl_poseCallback);
    ros::Subscriber line_downsample_points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front_camera/down_sample_line_points2", 1, pointCloudCallback);

    
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
    int macth_count_flg = 0;
 
    while (ros::ok())
    {
        time_stamp += DT; // dtで時間を積算
        for (int i = 0; i < PARTICLE_NUM; i++) 
        {
            if (particle_value[i] == 0.0) std::cout << "これでもゼロになった" << std::endl;
        }



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
        
        }



        
        if (downsampled_cloud.points.size() > 0) 
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
                for (int j = 0; j < downsampled_cloud.points.size(); j++)//検出した白線の座標配列
                {
                    // particle一つ一つについて絶対座標に変換
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

                        if (line_vertical_small < line_glx && line_glx < line_vertical_big && line_side_small < line_gly && line_gly < line_side_big)
                        {
                            match_count++;
                            break;
                        }

                    }
                }
                // std::cout << "全点群数  : " << downsampled_cloud.points.size() << "  マッチカウント数  : " << match_count << std::endl;
                if (match_count > downsampled_cloud.points.size())
                {
                    std::cout << "超えた！！！！！！！！！！！！！！！！！！！！！！！！！" << std::endl;
                }
                // 尤度の計算方法を確率
                likelihood_value = (1 - likely_coeff) * (double(match_count) / double(downsampled_cloud.points.size())) + likely_coeff; //尤度に微小数を追加
                particle_value[i] *= likelihood_value; //前回の重みｘ尤度＝今回の重み

                
                


                if (match_count != 0)
                {
                    macth_count_flg = 1;
                }

            }          
        }


        // ----------------推定値の出力：正規化→推定値→リサンプリング---------------------

        double max_value_p = particle_value[0];
        int max_value_index_p = 0;
        for (int i = 0; i < PARTICLE_NUM; i++)
        {

            if (max_value_p < particle_value[i])
            {
                max_value_p = particle_value[i];
                max_value_index_p = i;
            }

            // estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            // estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            // estimate_theta  += particle_value[i] * yaw;
        }
        std::cout << "max 尤度 " << particle_value[max_value_index_p]*500 << std::endl;


        likelihood_value_nomalization(); //正規化

        // 推定結果の自己位置
        double estimate_odom_x = 0.0;
        double estimate_odom_y = 0.0;
        double estimate_theta  = 0.0;

        // 推定値の算出：自己位置 +＝ 重み[i] * (x,y,theta)[i]　→重み付き平均から尤度の最大値を推定値に変更

        // //重みづけ平均 
        // for (int i = 0; i < PARTICLE_NUM; i++)
        // {
        //     double roll, pitch, yaw;
        //     geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[i].orientation); //yaw角に変換
        //     estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
        //     estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
        //     estimate_theta  += particle_value[i] * yaw;

        // }

// 最大値を尤度＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿￥
        double max_value = particle_value[0];
        int max_value_index = 0;
        for (int i = 0; i < PARTICLE_NUM; i++)
        {

            if (max_value < particle_value[i])
            {
                max_value = particle_value[i];
                max_value_index = i;
            }

            // estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            // estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            // estimate_theta  += particle_value[i] * yaw;
        }



        double roll, pitch, yaw;
        geometry_quat_to_rpy(roll, pitch, yaw, particle_cloud.poses[max_value_index].orientation); //yaw角に変換
        estimate_odom_x =   particle_cloud.poses[max_value_index].position.x;
        estimate_odom_y = particle_cloud.poses[max_value_index].position.y;
        estimate_theta  = yaw;
// ＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿￥
        estimate_posi.pose.pose.position.x = estimate_odom_x;
        estimate_posi.pose.pose.position.y = estimate_odom_y;
        estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, estimate_theta);

        ROS_INFO("v: %f, omega:%f",v, omega);


        // ----------------リサンプリングの処理開始----------------------
        
        if (v > 0.04 || std::fabs(omega) > 0.02)
        {
            
            if (macth_count_flg == 1) //
            {
                printf("cnt: %d, リサンプリング中\n",cnt);
                resampling(cnt);
                macth_count_flg = 0;
            }
            else 
            {
                std::cout << " match_coutはゼロ" << std::endl;

            }
        }
        else 
        {
            printf("cnt: %d, なにもしない\n",cnt);
        }
        


        cnt++;
        

        self_localization_pub.publish(posi);
        particle_cloud_pub.publish(particle_cloud);

        // estimate_posiだけ別の名前でTFを出力する
        
        tf::StampedTransform transform_esti_pose;
        transform_esti_pose.setOrigin(tf::Vector3(estimate_posi.pose.pose.position.x, estimate_posi.pose.pose.position.y, estimate_posi.pose.pose.position.z));
        transform_esti_pose.setRotation(tf::Quaternion(estimate_posi.pose.pose.orientation.x,estimate_posi.pose.pose.orientation.y,estimate_posi.pose.pose.orientation.z,estimate_posi.pose.pose.orientation.w));
        ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
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
