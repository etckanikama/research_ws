#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>
#include <random>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cstdlib>  // getenvを使うために必要
#include <sstream> // std::ostringstreamを使用するために必要
#include <cmath> // std::fmod関数のために必要


using namespace std;


ros::Time point_timestamp; //初期化
pcl::PointCloud<pcl::PointXYZ> downsampled_cloud; //センサー情報を受け取る
geometry_msgs::PoseArray particle_cloud; // poseArrayの値の入れ方：https://answers.ros.org/question/251586/rostopic-pub-geometry_msgsposearray-example/
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseWithCovarianceStamped estimate_posi,max_estimate_posi;

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

// 地図情報
float map_resolution = 0;
geometry_msgs::Pose map_origin;
int map_width = 0, map_height = 0;
std::vector<int8_t> map_data; // 地図データ


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
std::vector<double> particle_value(PARTICLE_NUM); // particleの重み


// pf_yaw:各パーティクルの姿勢、estimate_yaw:いち時刻前の推定姿勢（最初は初期値）
double normalizeYaw(double pf_yaw, double estimate_yaw) {
 //角度差を計算
 double delta = pf_yaw - estimate_yaw; 
  while(delta > M_PI){
    delta -= 2.0*M_PI;
  }
  while(delta < -M_PI){
    delta+=2.0*M_PI;
  }

  return estimate_yaw + delta;
}
// roll, pitch, yawからクォータニオンにする関数
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

// クォータニオンをオイラー角に変換
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat) {
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // rpy are Pass by Reference
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

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::fromROSMsg(*msg, downsampled_cloud);
}

// mapCallback関数の定義
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // 地図情報の更新
    map_resolution = msg->info.resolution;
    map_origin.position = msg->info.origin.position;
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_data = msg->data; // 地図データの更新

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

// センサー座標系の点群をグローバル座標系に変換
void transformPointCloudToGlobal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                                 const geometry_msgs::Pose& pose,
                                 pcl::PointCloud<pcl::PointXYZ>& output_cloud) {
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, pose.orientation);

    output_cloud.clear();
    for (const auto& point : input_cloud.points) {
        double line_glx = point.x * cos(yaw) - point.y * sin(yaw) + pose.position.x;
        double line_gly = point.x * sin(yaw) + point.y * cos(yaw) + pose.position.y;
        output_cloud.push_back(pcl::PointXYZ(line_glx, line_gly, point.z));
    }
}

int convertToMapFrame(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    if (map_resolution == 0) {
        ROS_WARN("Map information is not yet available.");
        return -1; // 地図情報がない場合はエラー値を返す
    }

    int match_count = 0; // 障害物（白線ピクセル）と一致した点の数

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        // 座標変換
        int pixel_x = (int)((cloud.points[i].x - map_origin.position.x) / map_resolution);
        int pixel_y = (int)((cloud.points[i].y - map_origin.position.y) / map_resolution);

        if (pixel_x >= 0 && pixel_x < map_width && pixel_y >= 0 && pixel_y < map_height) 
        {
            // ピクセルインデックスの計算
            int index = pixel_y * map_width + pixel_x;
            // ピクセルの値を取得
            int value = map_data[index];

            // ピクセルの値が100（障害物）の場合、match_countをインクリメント
            if (value == 100) {
                match_count++;
            }
            // 点の座標とピクセルの値（障害物、自由空間、または未知）を出力
            // cout << "Point " << i << " (x: " << cloud.points[i].x << ", y: " << cloud.points[i].y << ") ";
            // cout << "corresponds to pixel (" << pixel_x << ", " << pixel_y << ") ";
            // cout << "with value: " << value << endl;
        } 
        else 
        {
            cout << "Point " << i << " is outside the map boundaries." << endl;
        }
    }
    return match_count;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_corners_listener");
    ros::NodeHandle nh;
    // Publisher
    ros::Publisher self_localization_pub = nh.advertise<nav_msgs::Odometry>("self_position", 10); // robotにモータに指令値を送るpublisherの宣言。
    ros::Publisher particle_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 5); // リサンプリング後のパーティクル
    ros::Publisher esitimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate_position",5);// posewithcoverianceで推定値をpublish
    ros::Publisher max_estimate_position_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("max_estimate_position",5);// posewithcoverianceで推定値をpublish

    // Subscriber
    ros::Subscriber sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front_camera/down_sample_line_points2", 1, pointCloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/beego/diff_drive_controller/odom",10, velCallback);//gazeboを挟んだオドメトリ/beego/diff_drive_controller/odom をサブスクライブ


    particle_cloud.header.frame_id = "map";
    particle_cloud.header.stamp = ros::Time::now();
    particle_cloud.poses.resize(PARTICLE_NUM);

    double init_x=0.0,init_y=0.0,init_yaw=0.0;
    // double init_x = 10.7, init_y = -3.3, init_yaw = 1.57; // 初期値


    nav_msgs::Odometry posi;
    tf::TransformBroadcaster broadcaster;

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

    ros::Rate rate(10.0); // 10Hzでループを回す
    int cnt = 0;
    double time_stamping = 0.0;
    ros::Time start= ros::Time::now();
    std::cout << "Start time: " << start.toSec() << std::endl;

    while (ros::ok()) 
    {
        likelihood_state = 0; // 尤度更新はデフォルトはしない

        ros::spinOnce(); // コールバック関数を実行

        ros::Time current = ros::Time::now();
        ros::Duration elapsed = current - start;
        double DT = elapsed.toSec();
        if (DT==0 || DT > 130) DT =  1.0 / 10.0; //最初だけ0と意味わからん数値が入るやつを防ぐ

        double likelyhood_array[PARTICLE_NUM]; //尤度を格納する配列
        double max_likelyhood = 0; // 最大尤度
        double sum_likelyhood = 0; //尤度の総和
        // double DT = 1 / s;  
        time_stamping += DT; // dtで時間を積算
        cout << "時間 " << time_stamping << endl;


        //----------------------------- パーティクルの位置に状態遷移モデルの適用------------------------------------------------------------
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
        // -----------------------------------------------------------------------------------------
        
        //---------------------------------------- 尤度の計算条件--------------------------------------------------------
        ros::Time now = ros::Time::now();// 現在時刻の取得と表示
        ros::Duration diff = now - point_timestamp;
        cout << "点群の遅延時間: " << diff.toSec() << " seconds" << endl;
        
        // if (diff.toSec() < DelayThreshold) //遅延許容誤差：0.2以上の遅延では尤度計算を行わない
        // {
        cout << "全ての白線点群数 "  << downsampled_cloud.points.size() << endl;

        if (!downsampled_cloud.points.empty()) 
        {
            likelihood_state = 1;// 尤度更新を行う状態にセット

            // 尤度計算用の変数
            double likelihood_value    = 0;
            // 変換された点群データの処理
            for (int i = 0; i < PARTICLE_NUM; ++i) 
            {
                pcl::PointCloud<pcl::PointXYZ> global_pointcloud;
                // グローバル座標に白線点群を変換
                transformPointCloudToGlobal(downsampled_cloud, particle_cloud.poses[i], global_pointcloud);// パーティクルを使って点群座標をグローバル座標に変換
                
                int match_count = convertToMapFrame(global_pointcloud);
                // cout << "パーティクル：" << i << " のときのmatch_count " << match_count << endl; 
                
                likelihood_value = (1 - coeff)*(double(match_count) / double(downsampled_cloud.points.size())) + coeff; // m/M今まで
                particle_value[i] *= likelihood_value; //前回の重みｘ尤度＝今回の重み
                likelyhood_array[i] = likelihood_value; //尤度配列に尤度を格納
            }

        }
        // }
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

        // 重みの正規化 : particle_valueの値を正規化
        likelihood_value_nomalization();

        // 重み付け平均による推定値の算出
        // 重みづけ平均の推定結果の自己位置の初期化-----------------------------------------------
        double estimate_odom_x = 0.0;
        double estimate_odom_y = 0.0;
        double estimate_theta  = 0.0;

        // 重みづけ平均：自己位置を出す処理：自己位置 +＝ 重み[i] * (x,y,theta)[i]ーーーーーーーーーーーーーーーーーーーーー
        for (int i = 0; i < PARTICLE_NUM; i++)
        {
            double roll, pitch, pf_yaw;
            geometry_quat_to_rpy(roll, pitch, pf_yaw, particle_cloud.poses[i].orientation); //yaw角に変換
            pf_yaw = normalizeYaw(pf_yaw, estimate_theta); // normalizeYaw(各パーティクルの姿勢, いち時刻前の推定姿勢)
            estimate_odom_x += particle_value[i] * particle_cloud.poses[i].position.x;
            estimate_odom_y += particle_value[i] * particle_cloud.poses[i].position.y;
            estimate_theta  += particle_value[i] * pf_yaw;
            // cout <<"particle_value_ "<< i << " :" << particle_value[i] <<" "<< "各パーティクルのyaw_" << i  << ":  "<< yaw << endl;

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

        double roll, pitch, max_yaw;
        geometry_quat_to_rpy(roll, pitch, max_yaw, particle_cloud.poses[max_value_index].orientation); //yaw角に変換
        max_estimate_odom_x =   particle_cloud.poses[max_value_index].position.x;
        max_estimate_odom_y = particle_cloud.poses[max_value_index].position.y;
        max_estimate_theta  = max_yaw;

        max_estimate_posi.pose.pose.position.x = max_estimate_odom_x;
        max_estimate_posi.pose.pose.position.y = max_estimate_odom_y;
        max_estimate_posi.pose.pose.orientation = rpy_to_geometry_quat(0,0, max_estimate_theta);
        // ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

        // std::cout << "真値 x " << beego_x << " " << "真値 y " << beego_y << " " <<"真値 yaw " << beego_yaw <<std::endl; 
        std::cout << "重み付け推定値 x " << estimate_odom_x << " "<< "重み付け推定値 y " << estimate_odom_y << " " << "重み付け平均 yaw " << estimate_theta <<std::endl;
        std::cout << "max推定値 x " << max_estimate_odom_x << " "<< "max推定値 y " << max_estimate_odom_y << " " << "max推定値 yaw " << max_estimate_theta <<  std::endl;
        std::cout << "オドメトリ_x " << robot_x << " "<< "オドメトリ_y " << robot_y << " " << "オドメトリ_yaw " << robot_yaw << std::endl;
        std::cout << "v  " << v << " " << "omega " << omega << std::endl;
        std::cout << "最大の重み " << particle_value[max_value_index] << std::endl;
        
        // csvなどに推定値を出力

        // リサンプリング
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

        // ------------transformaer-------------------
        tf::StampedTransform transform_esti_pose;
        transform_esti_pose.setOrigin(tf::Vector3(estimate_posi.pose.pose.position.x, estimate_posi.pose.pose.position.y, estimate_posi.pose.pose.position.z));
        transform_esti_pose.setRotation(tf::Quaternion(estimate_posi.pose.pose.orientation.x,estimate_posi.pose.pose.orientation.y,estimate_posi.pose.pose.orientation.z,estimate_posi.pose.pose.orientation.w));
        // ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
        transform_esti_pose.stamp_ = ros::Time::now();
        transform_esti_pose.frame_id_ = "map";
        transform_esti_pose.child_frame_id_ = "grid_dha_esti_pose";
        broadcaster.sendTransform(transform_esti_pose);

        geometry_msgs::PoseWithCovarianceStamped estimate_posi;
        estimate_posi.header.frame_id = "grid_dha_esti_pose";
        esitimate_position_pub.publish(estimate_posi);
        max_estimate_position_pub.publish(max_estimate_posi);



        start = current; // 次のループのために現在時刻を更新
        likelyhood_array[maxIndex] = 0.0; //尤度の格納配列を0で初期化
        rate.sleep();
        cnt++;
    }

    return 0;
}
