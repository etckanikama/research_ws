#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import ast

"rviz可視化用プログラム"


def publish_coordinates():
    rospy.init_node('visualize_candidate_f', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    
    # F はtextで与えられる中身は辞書型
    # ファイルから辞書を読み込む
    with open("/home/hirayama-d/research_ws/src/map_optimization/scripts/F.text", "r") as file:
        data = file.read()
        F = ast.literal_eval(data[data.find('{'):])  # 辞書部分を抽出して変換

    while not rospy.is_shutdown():
        for key, (x, y,state) in F.items():
            if rospy.is_shutdown():
                break  # シャットダウン要求があればループから抜ける
            
            print(key,x,y)
            marker = Marker()
            marker.header.frame_id = "beego/odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0  
            # クォータニオンをアイデンティティとして設定
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = int(key[1:])  # キーからIDを生成

            pub.publish(marker)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_coordinates()
    except rospy.ROSInterruptException:
        pass