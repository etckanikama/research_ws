#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy 
import numpy as np
import math
from	geometry_msgs.msg	import PoseStamped, Point, Quaternion, Point32
from sensor_msgs.msg import PointCloud


# right_lane


class Likelihood:

	def __init__(self):
		print("\n========== 尤度計算 ===============")
		rospy.init_node("likelihood")

        # Subscriber -----------------------------------------------------------
		self.line_points_sub = rospy.Subscriber('/front_camera/line_points', PointCloud, self.cbPoint)
		self.lane_position = PointCloud()
		# Publisher -----------------------------------------------------------
		self.line_points_conversion_pub = rospy.Publisher('/front_camera/line_points_conversion', PointCloud, queue_size=1)
		

	def cbPoint(self, data):
		self.lane_position = data


	


def main():
	node	= Likelihood()
	rate = rospy.Rate(10)
	line_x_under = 1.25
	line_x_top = 3.0

	# ロボットの初期姿勢(今後はオドメトリからsubしてくる予定)
	robot_x = 0
	robot_y = 0 #m
	robot_yaw = 0 #rad
	# x_bottom, x_top, y_right, y_left
	line_map = [[1.25, 3.0, 0.475, 0.525],[1.25, 2.0, -0.475, -0.525]]
	while not rospy.is_shutdown():
		lp = node.lane_position.points #pclから受け取った白線の座標情報
		# lp_x = 1.0
		# lp_y = 0.0
		match_count = 0
		# line_glx = lp_x * math.cos(robot_yaw) - lp_y * math.sin(robot_yaw) + robot_x
		# line_gly = lp_x * math.sin(robot_yaw) + lp_y * math.cos(robot_yaw) + robot_y
		# print(line_glx, line_gly)
		the_points		= PointCloud()
		the_points.header.frame_id	= 'front_realsense_link'
		print("pointの数", len(lp))

		if len(lp) > 0:
			print("robot_x: {} cm, robot_y: {} cm, robot_yaw: {} deg".format(robot_x, robot_y*100, robot_yaw*180/math.pi))
			print("laneの座標の数",len(lp))
			print("line_0の座標",lp[0].x)
			for i in range(len(lp)):
				the_p		= Point32()
				# 絶対座標への変換
				line_glx = lp[i].x * math.cos(robot_yaw) - lp[i].y * math.sin(robot_yaw) + robot_x
				line_gly = lp[i].x * math.sin(robot_yaw) + lp[i].y * math.cos(robot_yaw) + robot_y
				# line_glx = lp_x * math.cos(robot_yaw) - lp_y * math.sin(robot_yaw) + robot_x
				# line_gly = lp_x * math.sin(robot_yaw) + lp_y * math.cos(robot_yaw) + robot_y
				# 座標変換確認用
				the_p.x = line_glx
				the_p.y = line_gly
				the_p.z = 0.0
				the_points.points.append(the_p)
				# print(line_glx, line_gly)
		
				for j in range(len(line_map)):

					# 上下の小さい方か大きい方かを決める
					if line_map[j][0] < line_map[j][1]:
						line_vertical_small = line_map[j][0]
						line_vertical_big   = line_map[j][1]
					else:
						line_vertical_small = line_map[j][1]
						line_vertical_big   = line_map[j][0]						

					# 左右の小さい方か大きい方かを決める
					if line_map[j][2] < line_map[j][3]:
						line_side_small = line_map[j][2]
						line_side_big   = line_map[j][3]
					else:
						line_side_small = line_map[j][3]
						line_side_big   = line_map[j][2]
					print("qqqq{}, {}",line_glx, line_gly)
					# 上下は常に正とするプログラムになっている。上下に負が出た場合対応が必要
					# どっちが負とか関係なく小さい方を後ろ（Aとして決めて）にもってきて、常にA < x < Bでよくないか？
					if line_vertical_small < line_glx and line_glx < line_vertical_big  and line_side_small < line_gly and line_gly < line_side_big:
						match_count += 1

		print("マッチ数",match_count)
		# rospy.loginfo(node.pcl.points)
		node.line_points_conversion_pub.publish(the_points)
		rate.sleep()
	


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
