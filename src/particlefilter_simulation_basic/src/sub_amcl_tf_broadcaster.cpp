#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

// グローバル変数
double x = 0.0;
double y = 0.0;
geometry_msgs::Quaternion q;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // 位置と姿勢を取得
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    q = msg->pose.pose.orientation;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_amcl_tf_broadcaster");
    ros::NodeHandle nh;

    // amcl_poseのサブスクライバを作成し、コールバックを登録
    ros::Subscriber amcl_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/amcl_pose", 
        10, 
        amclPoseCallback
    );

    tf::TransformBroadcaster broadcaster;
    ros::Rate rate(10.0); // 10Hz

    while (ros::ok())
    {
        ros::spinOnce();  // コールバックを呼び出す

        // StampedTransformの作成と値の設定
        tf::StampedTransform transform;
        transform.frame_id_ = "map"; // amcl_poseは通常mapフレームに関連しています
        transform.child_frame_id_ = "custom_amcl_pose";
        transform.stamp_ = ros::Time::now();
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));

        // ブロードキャスト
        broadcaster.sendTransform(transform);

        // スリープ（次のイテレーションまで）
        rate.sleep();
    }

    return 0;
}
