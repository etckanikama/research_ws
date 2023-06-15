#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char** argv)
{
    // ROSの初期化
    ros::init(argc, argv, "pose_array_publisher");
    ros::NodeHandle nh;

    // パブリッシャの作成
    ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array_topic", 10);

    // ループの頻度を設定
    ros::Rate loop_rate(10);  // 10Hzの頻度でメッセージを発行

    while (ros::ok())
    {
        // geometry_msgs::PoseArrayメッセージの作成と設定
        geometry_msgs::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = ros::Time::now();
        pose_array_msg.header.frame_id = "map";
        pose_array_msg.poses.resize(3);  // パーティクル数を3とする（適宜変更）

        // 位置と姿勢の設定（適宜変更）
        pose_array_msg.poses[0].position.x = 0.0;
        pose_array_msg.poses[0].position.y = 1.0;
        pose_array_msg.poses[0].position.z = 0.0;
        pose_array_msg.poses[0].orientation.w = 0.0;

        pose_array_msg.poses[1].position.x = 0.0;
        pose_array_msg.poses[1].position.y = 3.0;
        pose_array_msg.poses[1].position.z = 0.0;
        pose_array_msg.poses[1].orientation.w = 0.0;

        pose_array_msg.poses[2].position.x = 0.0;
        pose_array_msg.poses[2].position.y = 5.0;
        pose_array_msg.poses[2].position.z = 0.0;
        pose_array_msg.poses[2].orientation.w = 0.0;

        // リサンプリングして、リサンプリングあとのarrayをpublish

        // メッセージのパブリッシュ
        pose_array_pub.publish(pose_array_msg);

        // コールバックや処理の実行
        ros::spinOnce();

        // ループの頻度になるように待機
        loop_rate.sleep();
    }

    return 0;
}
