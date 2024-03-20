#!/usr/bin/python3
# -*- coding: utf-8 -*-

#############################################################################

################################################################################
#                                   Import                                     #
################################################################################
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
import	cv2
import	random
import	tf.transformations
from	std_msgs.msg		import Bool, Float32MultiArray, Float64MultiArray
from	geometry_msgs.msg	import PoseStamped, Point, Quaternion, Point32
from	geometry_msgs.msg	import Pose2D, Twist, PoseArray, Pose
from	geometry_msgs.msg	import TransformStamped
from	sensor_msgs.msg		import Image, CameraInfo
from	sensor_msgs.msg		import PointCloud
from	cv_bridge			import CvBridge

from	sensor_msgs.msg		import PointCloud,PointCloud2,PointField
from std_msgs.msg import Header
import  sensor_msgs.point_cloud2 as pc2
################################################################################
#                             Grobal Definition                                #
################################################################################
# Image processing parameters ==================================================
# HSV range(L棟廊下のパラメータ:realsense)-----------------------------------------------------------
# H_LOW	= 0
# H_HIGH	= 20
# S_LOW	= 21
# S_HIGH	= 73
# V_LOW	= 120
# V_HIGH	= 255
# -----------------------------------------------------------------------------
# HSV range(SB棟10階のパラメータ:realsense)-----------------------------------------------------------
H_LOW	= 0
H_HIGH	= 62
S_LOW	= 0
S_HIGH	= 255
V_LOW	= 253
V_HIGH	= 255
# -----------------------------------------------------------------------------
# Median filter ----------------------------------------------------------------
MEDIAN_RGB_SIZE	= 9
MEDIAN_HSV_SIZE	= 3

# Point Cloud Skip Num ---------------------------------------------------------
SKIP_H_NUM		= 2
SKIP_W_NUM		= 4

# subscribe time parameters ====================================================
LINE_DETECTION_TIME		= 0.1				# 白線検出処理周期[s]
IMAGE_SUB_TIME_THRE		= 0.1				# Subscribe時間遅れ許容範囲[s] LINE_DETECTION_TIME以下の値

# 変換後のpointcloudのglobal座標を格納
points_list = []


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
		# the_src_pts		= np.array([(122.0, 33.0), (555.0, 23.0), (98.0, 353.0), (572.0, 355.0)], dtype=np.float32)
		# the_dst_pts		= np.array([(2.5, 1.0), (2.5, -1.0), (0.5, 0.15), (0.5, -0.15)], dtype=np.float32)
		# さきがけh:0.91,l=0.41パラメータ
		the_src_pts		= np.array([(5, 8), (632, 4), (5, 355.0), (636.0, 354.0)], dtype=np.float32)
		the_dst_pts		= np.array([(3.3, 2.0), (3.3, -1.8), (1.01, 0.7), (1.01, -0.67)], dtype=np.float32)
		self.H		= cv2.getPerspectiveTransform(the_src_pts, the_dst_pts)
		print("H=", self.H)





		# Subscriber -----------------------------------------------------------
#		self.front_rgb_image_sub	= rospy.Subscriber('/front_camera/image', Image, self.subFrontRGBImage)		# Front RGB Image

		# リアルセンスでtopicを使うとき
		self.front_rgb_image_sub	= rospy.Subscriber('/front_realsense/color/image_raw', Image, self.subFrontRGBImage)
		# webカメラをsubするとき
		# self.front_rgb_image_sub	= rospy.Subscriber('/head_camera/image_raw', Image, self.subFrontRGBImage)
#		self.rear_rgb_image_sub		= rospy.Subscriber('/rear_camera/image', Image, self.subRearRGBImage)		# Rear RGB Image


		# Publisher ------------------------------------------------------------
		self.front_hsv_image_pub				= rospy.Publisher('/front_camera/hsv_image', Image, queue_size=1)				# Front HSV Image
		self.front_hsv_mask_image_pub			= rospy.Publisher('/front_camera/hsv_mask_image', Image, queue_size=1)			# Front HSV Mask Image
		self.front_rgb_median_image_pub			= rospy.Publisher('/front_camera/rgb_median_image', Image, queue_size=1)		# Front RGB Median Image
		self.front_hsv_median_image_pub			= rospy.Publisher('/front_camera/hsv_median_image', Image, queue_size=1)		# Front HSV Median Image(after RGB median)
		self.front_hsv_median_mask_image_pub	= rospy.Publisher('/front_camera/hsv_median_mask_image', Image, queue_size=1)	# Front HSV Median Mask Image(after RGB median)
		self.line_points_pub					= rospy.Publisher('/front_camera/line_points', PointCloud, queue_size=1)		# Line Points
		self.line_points_pc2_pub					= rospy.Publisher('/front_camera/line_points_pc2', PointCloud2, queue_size=1)

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
				# -----------一応、min_area_thresholdのpxel数以下の255以上の数値を消す処理
				contours, _ = cv2.findContours(self.front_hsv_median_mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
				min_area_threshold = 2000  # Adjust this threshold as needed
				self.front_hsv_median_mask_img = np.zeros_like(self.front_hsv_median_mask_img)
				for contour in contours:
					area = cv2.contourArea(contour)
					if area >= min_area_threshold:
						cv2.drawContours(self.front_hsv_median_mask_img, [contour], -1, 255, thickness=cv2.FILLED)
				# ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー


				# Print out the total area of white regions and the path to the cleaned image
				total_white_area = np.sum(self.front_hsv_median_mask_img / 255)  # Convert to a binary scale for area calculation
				# print(total_white_area)
				# the_points.header.frame_id	= 'front_realsense_link'
				# the_points.header.frame_id	= 'dha_esti_pose'
				# the_points.header.frame_id	= 'custom_amcl_pose'
				# the_points.header.frame_id	= 'hdl_pose' # 地図に貼り付けるときはmap座標系:map
				header = Header()
				header.stamp = rospy.Time.now()
				header.frame_id = 'hdl_pose'
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
							if x <= 3.0:
								the_points_array.append([x,y,z])	
						# if self.front_hsv_median_mask_img[the_v, the_u] > 250:
						# 	the_p		= Point32()
						# 	the_p.x		= (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
						# 	the_p.y		= (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
						# 	the_p.z		= 0.0
						# 	if the_p.x <= 3.0:
						# 		the_points.points.append(the_p)
								# point_dict = {'x': the_p.x, 'y': the_p.y, 'z': the_p.z}
								# points_list.append(point_dict)
								# print(the_p)	
				
				# print(points_list)
				#---------------------------------------------------------------
				#		Publish line points & images
				#---------------------------------------------------------------
				# white line point publish
				# self.line_points_pub.publish(the_points)
				line_msg = pc2.create_cloud_xyz32(header, the_points_array)
				line_msg.fields = fields
				self.line_points_pc2_pub.publish(line_msg)
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
