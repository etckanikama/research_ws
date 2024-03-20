#!/usr/bin/env python3
import rospy

"""
ゴール：白線点群をグローバル座標に変換すること
subscribe
・自己位置推定値:/hdl_odom

transformerを出力して,親フレームがmapのhdl_odomの推定値を子フレーム（適当な名前）に持つ
点群を出力してみるか。。。
それを
"""


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTFBroadcaster:
    def __init__(self):
        rospy.init_node('odom_to_tf_broadcaster', anonymous=True)

        # オドメトリデータの購読
        rospy.Subscriber('/hdl_odom', Odometry, self.callback_hdl_odom)

        # tfブロードキャスターの初期化
        self.br = tf2_ros.TransformBroadcaster()

    def callback_hdl_odom(self, msg):
        # TransformStampedメッセージの作成
        t = TransformStamped()

        # メッセージのヘッダ設定
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "hdl_pose"

        # オドメトリデータから位置と姿勢を設定
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # tfのブロードキャスト
        self.br.sendTransform(t)

if __name__ == '__main__':
    try:
        odom_to_tf = OdomToTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    
