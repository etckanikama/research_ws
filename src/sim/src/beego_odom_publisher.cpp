#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h> // tfライブラリのインクルード

// グローバル変数
double init_x = 10.7, init_y = -3.3, init_yaw = 1.57; // 初期値
// double init_x = 0.0, init_y = 0.0, init_yaw = 0.0; // 初期値
ros::Publisher cloud_pub; // グローバル変数としてパブリッシャーを宣言
ros::Publisher pose_with_cov_pub; // PoseWithCovarianceStamped メッセージ用のパブリッシャー
ros::Publisher pose_pub; // robot_pose トピック用のパブリッシャー
tf::TransformBroadcaster *br;

double normalizeYaw(double yaw) {
    return atan2(sin(yaw), cos(yaw));
}
// Odometry メッセージのコールバック関数
void velCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // オドメトリからの相対位置と姿勢を取得
    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, odom_yaw;
    m.getRPY(roll, pitch, odom_yaw); // Yaw 角を取得

    // グローバル座標の計算
    double global_x = init_x + (odom_x * cos(init_yaw) - odom_y * sin(init_yaw));
    double global_y = init_y + (odom_x * sin(init_yaw) + odom_y * cos(init_yaw));
    double global_yaw = normalizeYaw(init_yaw + odom_yaw);  

    // グローバル座標での PoseWithCovarianceStamped メッセージの作成
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map"; // map フレームを設定
    pose_msg.pose.pose.position.x = global_x;
    pose_msg.pose.pose.position.y = global_y;
    tf::Quaternion q_global = tf::createQuaternionFromYaw(global_yaw);
    pose_msg.pose.pose.orientation.x = q_global.x();
    pose_msg.pose.pose.orientation.y = q_global.y();
    pose_msg.pose.pose.orientation.z = q_global.z();
    pose_msg.pose.pose.orientation.w = q_global.w();

    // メッセージを robot_pose トピックに publish
    pose_pub.publish(pose_msg);
}

// Gazebo Model States コールバック関数
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "beego") {
            geometry_msgs::Point32 point;
            point.x = msg->pose[i].position.x;
            point.y = msg->pose[i].position.y;
            point.z = 0; // 2D ポイントクラウドのため Z は 0
            cloud.points.push_back(point);
            std::cout << "gazebo_beego x: " << point.x << ", gazebo_beego y: " << point.y << std::endl;
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "map"; // map フレームを設定
            pose_msg.pose.pose = msg->pose[i]; // 位置と姿勢を設定
            pose_msg.pose.pose.position.z = 0;//z軸は常に０で表示する（強制的に）
            // メッセージを publish
            pose_with_cov_pub.publish(pose_msg);

            // tfトランスフォーメーションの送信
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose[i].position.x, msg->pose[i].position.y, 0.0));
            tf::Quaternion q(
                msg->pose[i].orientation.x,
                msg->pose[i].orientation.y,
                msg->pose[i].orientation.z,
                msg->pose[i].orientation.w);
            transform.setRotation(q);

            br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "beego_frame"));

            break;
        }
    }

    cloud_pub.publish(cloud);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "beego_odom_publisher");
    ros::NodeHandle nh;

    // tfブロードキャスターの初期化
    br = new tf::TransformBroadcaster();

    // Odometry トピックのサブスクライバー
    ros::Subscriber odom_sub = nh.subscribe("/beego/diff_drive_controller/odom", 10, velCallback);

    // 新しいトピックのパブリッシャー
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 10);
    pose_with_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("beego_pose_with_cov", 10);
    
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("beego_point_cloud", 10);

    // 10Hz のレートでループ
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // // 現在の robot_x, robot_y, yaw を使ってメッセージを作成
        // geometry_msgs::PoseWithCovarianceStamped pose_msg;
        // pose_msg.header.stamp = ros::Time::now();
        // pose_msg.header.frame_id = "map"; // map フレームを設定
        // pose_msg.pose.pose.position.x = robot_x;
        // pose_msg.pose.pose.position.y = robot_y;
        // tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        // pose_msg.pose.pose.orientation.x = q.x();
        // pose_msg.pose.pose.orientation.y = q.y();
        // pose_msg.pose.pose.orientation.z = q.z();
        // pose_msg.pose.pose.orientation.w = q.w();
        // std::cout << "オドメトリ x " << robot_x << " " << "オドメトリ y " << robot_y << std::endl;

        // // メッセージを publish
        // pose_pub.publish(pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    delete br; // tfブロードキャスターの削除

    return 0;
}
