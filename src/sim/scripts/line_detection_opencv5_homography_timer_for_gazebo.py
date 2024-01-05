#!/usr/bin/env python3
import	sys
import	rospy
import	copy
import	tf
import	std_msgs
import	sensor_msgs
import	geometry_msgs
import	math
import	time
import	numpy as np
import open3d as o3d
import	cv2
import	random
import	tf.transformations
from	std_msgs.msg		import Bool, Float32MultiArray, Float64MultiArray
from	geometry_msgs.msg	import PoseStamped, Point, Quaternion, Point32
from	geometry_msgs.msg	import Pose2D, Twist, PoseArray, Pose
from	geometry_msgs.msg	import TransformStamped
from	sensor_msgs.msg		import Image, CameraInfo
from	sensor_msgs.msg		import PointCloud,PointCloud2,PointField
from	cv_bridge			import CvBridge
from std_msgs.msg import Header
import  sensor_msgs.point_cloud2 as pc2

################################################################################
#                             Grobal Definition                                #
################################################################################

# -----------------------------------------------------------------------------
# HSV range(SB棟10階のパラメータ:realsense)-----------------------------------------------------------
H_LOW	= 0
H_HIGH	= 255
S_LOW	= 0
S_HIGH	= 150
V_LOW	= 122
V_HIGH	= 255
# -----------------------------------------------------------------------------
# Median filter ----------------------------------------------------------------
MEDIAN_RGB_SIZE	= 1
MEDIAN_HSV_SIZE	= 3
# MEDIAN_RGB_SIZE	= 9
# MEDIAN_HSV_SIZE	= 3

# Point Cloud Skip Num ---------------------------------------------------------
SKIP_H_NUM		= 1
SKIP_W_NUM		= 4

# subscribe time parameters ====================================================
LINE_DETECTION_TIME		= 0.1				# 白線検出処理周期[s]
IMAGE_SUB_TIME_THRE		= 0.1				# Subscribe時間遅れ許容範囲[s] LINE_DETECTION_TIME以下の値

# ボクセルダウンサンプリングのパラメータ
VOXEL_SIZE = 0.02  # ボクセルサイズを設定:2cm間隔でダウンサンプリングをした


#################################################################################
#								Functions										#
#################################################################################
def limitAnglePi(in_angle):
	return (in_angle + math.pi)%(2.0*math.pi) - math.pi



###############################################################################
#								Line detection								#
###############################################################################
class LineDetection:

	#===========================================================================
	#   Constructor
	#===========================================================================
	def __init__(self):
		print ("\n=============== Line Detection HSV ====================")

		# Initialize node ------------------------------------------------------
		rospy.init_node("line_detection")


		# Valuables ------------------------------------------------------------
		# openCV ----------------------------
		self.bridge				= CvBridge()
		self.front_rgb_sub_flg	= False			# 画像のsubscribe有無
		self.front_rgb_sub_time	= rospy.get_time()


		# HSV range -------------------------
		self.h_low			= rospy.get_param('~h_low',  H_LOW)
		self.h_high			= rospy.get_param('~h_high', H_HIGH)
		self.s_low			= rospy.get_param('~s_low',  S_LOW)
		self.s_high			= rospy.get_param('~s_high', S_HIGH)
		self.v_low			= rospy.get_param('~v_low',  V_LOW)
		self.v_high			= rospy.get_param('~v_high', V_HIGH)
		self.hsv_low		= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
		self.hsv_high		= np.array([self.h_high, self.s_high, self.v_high], np.uint8)

		# Median filter size ----------------
		self.median_rgb_size	= rospy.get_param('~median_rgb_size', MEDIAN_RGB_SIZE)
		self.median_hsv_size	= rospy.get_param('~median_hsv_size', MEDIAN_HSV_SIZE)

		# Homograpy Matrix (DHAパラメータ) ---
		# the_src_pts		= np.array([(66.0, 17.0), (548.0, 9.0), (138.0, 304.0), (592.0, 303.0)], dtype=np.float32)
		# the_dst_pts		= np.array([(1.5, 0.75), (1.5, -0.5), (0.5, 0.15), (0.5, -0.15)], dtype=np.float32)
		# self.H		= cv2.getPerspectiveTransform(the_src_pts, the_dst_pts)
        # Homograpy 行列の作成：俯角：0.5rad(28.64deg)---------------
		# cv_points		= np.array([(44.0, 18.0), (603.0, 19.0), (634.0, 338.0), (6.0, 336.0)], dtype=np.float32)
		# ros_points		= np.array([(4.3, 2.3), (4.3, -2.3), (0.58, -0.24), (0.58, 0.25)], dtype=np.float32)
		# cv_points		= np.array([(6.0, 7.0), (638.0, 7.0), (640.0, 349.0), (3.0, 346.0)], dtype=np.float32)
		# ros_points		= np.array([(3.35, 1.97), (3.35, -1.97), (0.55, -0.23), (0.55, 0.24)], dtype=np.float32)
		# # Homograpy 行列の作成：俯角：0.418879rad(24deg)---------------＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿
		# # cv_points		= np.array([(5.0, 2.0), (637.0, 5.0),(5.0, 358.0), (637.0, 358.0)], dtype=np.float32)
		# cv_points		= np.array([(0.0, 0.0), (640.0, 0.0),(640.0, 360.0),(0.0, 360.0)], dtype=np.float32)
		# ros_points	= np.array([(4.65, 2.8), (4.65, -2.7), (0.51, -0.21),(0.51, 0.19)], dtype=np.float32)
		# # Homograpy 行列の作成：俯角：0.3926991rad(22.5deg)---------------＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿
		cv_points		= np.array([(0.0, 0.0), (640.0, 0.0),(640.0, 360.0),(0.0, 360.0)], dtype=np.float32)
		# cv_points		= np.array([(0.0, 0.0), (1280.0, 0.0),(1280.0, 720.0),(0.0, 720.0)], dtype=np.float32)
		# ros_points	= np.array([(9.97, 6.2), (9.97, -6.2), (0.52, -0.21),(0.52, 0.18)], dtype=np.float32)
		ros_points	= np.array([(10.3, 6.5), (10.3, -6.5), (0.52, -0.21),(0.52, 0.2)], dtype=np.float32)
		self.H		= cv2.getPerspectiveTransform(cv_points, ros_points)
		# ＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿＿￥
		# 俯角：23deg
		print("H= ", self.H)


		# Subscriber -----------------------------------------------------------
		#シミュレーションのカメラ
		# self.image_sub = rospy.Subscriber("/beego/my_robo/camera1/image_raw", Image, self.subFrontRGBImage, queue_size=1)
		self.image_sub = rospy.Subscriber("/camera2/color/image_raw", Image, self.subFrontRGBImage, queue_size=1)


		# Publisher ------------------------------------------------------------
		# self.front_hsv_image_pub				= rospy.Publisher('/front_camera/hsv_image', Image, queue_size=1)				# Front HSV Image
		# self.front_hsv_mask_image_pub			= rospy.Publisher('/front_camera/hsv_mask_image', Image, queue_size=1)			# Front HSV Mask Image
		# self.front_rgb_median_image_pub			= rospy.Publisher('/front_camera/rgb_median_image', Image, queue_size=1)		# Front RGB Median Image
		# self.front_hsv_median_image_pub			= rospy.Publisher('/front_camera/hsv_median_image', Image, queue_size=1)		# Front HSV Median Image(after RGB median)
		self.front_hsv_median_mask_image_pub	= rospy.Publisher('/front_camera/hsv_median_mask_image', Image, queue_size=1)	# Front HSV Median Mask Image(after RGB median)
		self.line_points_pub					= rospy.Publisher('/front_camera/line_points_1', PointCloud2, queue_size=1)		# Line Points
		self.line_points_downsampling_pub       = rospy.Publisher('/front_camera/down_sample_line_points2',PointCloud2, queue_size=10)
		# self.line_points_downsampling_pub = rospy.Publisher('/front_camera/line_points2',PointCloud2, queue_size=10)

		# ---------------------------------------------------------

		# Timer func ------------------------------------------------------------
		rospy.Timer(rospy.Duration(LINE_DETECTION_TIME), self.lineDetection)

		rospy.spin()


	#===========================================================================
	#   GUI Timer Function
	#===========================================================================
	def lineDetection(self, in_dummy):

		if self.front_rgb_sub_flg == True:

			# Image subscribe time check ------------------------
			#the_now		= rospy.Time.now()
			#print "time:", the_now.to_sec() - self.front_rgb_sub_time.to_sec()
			#if (the_now.to_sec() - self.front_rgb_sub_time.to_sec()) < IMAGE_SUB_TIME_THRE:
			the_now 	= rospy.get_time()
			if (the_now - self.front_rgb_sub_time) < IMAGE_SUB_TIME_THRE:

				#---------------------------------------------------------------
				#		Color filtering & masking
				#---------------------------------------------------------------

				# rgb median
				self.front_rgb_median_img	= cv2.medianBlur(self.front_rgb_img, self.median_rgb_size)

				# hsv median
				self.front_hsv_median_img	= cv2.cvtColor(self.front_rgb_median_img, cv2.COLOR_BGR2HSV)

				# hsv median mask with range
				self.front_hsv_median_mask_img	= cv2.inRange(self.front_hsv_median_img, self.hsv_low, self.hsv_high)


				#---------------------------------------------------------------
				#		Homograpy transform
				#---------------------------------------------------------------
				# http://opencv.jp/opencv-2.1/cpp/geometric_image_transformations.html#warpPerspective
				the_color_h		= self.front_hsv_median_mask_img.shape[0]
				the_color_w		= self.front_hsv_median_mask_img.shape[1]

				# the_points		= PointCloud()
				# the_points.header.frame_id	= 'beego/odom'
				header = Header()
				header.stamp = rospy.Time.now()
				# header.frame_id = "fixed_paritcle_posi" #尤度分布用
				header.frame_id = "dha_esti_pose"
				# header.frame_id	= 'beego/odom'
				fields = [
					PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            		PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            		PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        		]

				the_points_array = []
				for the_v in range(0, the_color_h, SKIP_H_NUM):
					for the_u in range(0, the_color_w, SKIP_W_NUM):
						if self.front_hsv_median_mask_img[the_v, the_u] > 250:
							x = (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
							y = (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
							z = 0.0
							if x <= 2.5:
								the_points_array.append([x,y,z])
								# print(x,y)							
							# the_p		= Point32()
							# the_p.x		= (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
							# the_p.y		= (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
							# the_p.z		= 0.0
							# the_points.points.append(the_p)
							# print(the_p)

				# ダウンサンプリングする前の点群をpublish
				line_msg = pc2.create_cloud_xyz32(header, the_points_array)
				line_msg.fields = fields
				self.line_points_pub.publish(line_msg)

				# open3dによるボクセルダウンサンプリング 
				pcd = o3d.geometry.PointCloud()
				pcd.points = o3d.utility.Vector3dVector(the_points_array)
				downsampled = pcd.voxel_down_sample(VOXEL_SIZE)
				downsampled_points = []
				for point in downsampled.points:
					downsampled_points.append((point[0],point[1],point[2]))
					# print(point[0],point[1])
				# print("x:{0}, y{1}".format(downsampled_points[0],downsampled_points[0]))
				# print(downsampled_points)
				downsampled_msg = pc2.create_cloud_xyz32(header, downsampled_points)
				downsampled_msg.fields = fields
				# PointCloud2メッセージを作成
				# cloud_msg = pc2.create_cloud_xyz32(header, the_points_array)
				# cloud_msg.fields = fields
				# print("cloud_msg:",type(cloud_msg))
				self.line_points_downsampling_pub.publish(downsampled_msg)

				#---------------------------------------------------------------
				#		Publish line points & images
				#---------------------------------------------------------------
				# white line point publish
				# self.line_points_pub.publish(the_points)
				# hsv mask image
				# the_hsv_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_mask_img, encoding="mono8")
				# self.front_hsv_mask_image_pub.publish(the_hsv_mask_msg)
				# hsv median mask image
				the_hsv_median_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_median_mask_img, encoding="mono8")
				self.front_hsv_median_mask_image_pub.publish(the_hsv_median_mask_msg)



	#===========================================================================
	#   Get Front RGB Image
	#===========================================================================
	def subFrontRGBImage(self, in_rgb):

		# Store current front rgb image ----------------------------------------
		self.front_rgb_sub_flg	= True
		#self.front_rgb_sub_time	= in_rgb.header.stamp	# time stamp use_sim_time true: required for rosbag play
		self.front_rgb_sub_time	= rospy.get_time()
		self.front_rgb_img		= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')




################################################################################
#                               Main Function                                  #
################################################################################
if __name__ == '__main__':

	line_detection	= LineDetection()
    