#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// yaw角をtfのクォータニオン型に変換する関数
tf::Quaternion getYawQuaternion(double yaw)
{
    tf::Quaternion q;
    q.setRPY(0, 0, yaw); // RPY stands for Roll, Pitch, Yaw
    return q;
}

int main(int argc, char **argv) {
    // ROS ノードの初期化
    ros::init(argc, argv, "pose_array_publisher");
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
        ROS_WARN("Not argumment. Use default value");
        param_global_init_x   = 0.0;
        param_global_init_y   = 0.0;
        param_global_init_yaw = 0.0;
    }
    // -----------------------------------------------------------------------------


    // パブリッシャーの作成
    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_d", 10);


    tf::TransformBroadcaster broadcaster; //tfのブロードキャスト
    
    
    // ループのレートを設定
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // コールバックの処理
        ros::spinOnce();

        // PoseArray メッセージのインスタンスを作成
        geometry_msgs::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = ros::Time::now();

        // 単一のパーティクルのポーズを作成
        geometry_msgs::Pose pose;
        pose.position.x = param_global_init_x;
        pose.position.y = param_global_init_y;
        pose.position.z = 0.0; // Z は通常 0 で設定
        pose.orientation = tf::createQuaternionMsgFromYaw(param_global_init_yaw);

        // PoseArray にポーズを追加
        pose_array_msg.poses.push_back(pose);



        // tfのtransformerrを追加する
        // 白線点群
        tf::StampedTransform transform_particle_pose;
        transform_particle_pose.setOrigin(tf::Vector3(param_global_init_x, param_global_init_y, 0));
        transform_particle_pose.setRotation(getYawQuaternion(param_global_init_yaw));
        // ROS_INFO_STREAM(estimate_posi.pose.pose.position.x);    
        transform_particle_pose.stamp_ = ros::Time::now();
        transform_particle_pose.frame_id_ = "beego/odom";
        transform_particle_pose.child_frame_id_ = "fixed_paritcle_posi";
        broadcaster.sendTransform(transform_particle_pose);

        pose_array_msg.header.frame_id = "beego/odom"; // 適切なフレームIDに設定
        pose_array_pub.publish(pose_array_msg); //publish



        // ループの待機
        loop_rate.sleep();
    }



    return 0;
}
