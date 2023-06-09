#!/usr/bin/python3
# -*- coding: utf-8 -*-
#############################################################################
#########################	UNIVERSITY OF TSUKUBA	#########################
#############################################################################
################################  Python  ###################################
#############################################################################
#############################################################################
##########  Intelligent Robot Lab., Department of Computer Science  #########
#############################################################################
#############################################################################
###                      line_detection_opencv.py                         ###
###                                                                       ###
###          composed by Ayanori Yorozu                                   ###
###          at Intelligent Robot Laboratory, University of Tsukuba       ###
###                                                                       ###
###                           Copyright (2022), All rights reserved.      ###
###                                                                       ###
###          c/o. Faculty of Engineering, Information and Systems,        ###
###          1-1-1 Tennodai, Tsukuba, Ibaraki 305-8573                    ###
###          E-mail: yorozu@cs.tsukuba.ac.jp                              ###
#############################################################################
#############################################################################
# 2022.10.02
# HSV Median Maskで検出した白線バイナリ画像に対して，
# 白のpixel座標をH行列で変換し，ロボット座標(front_realsense_link(仮))でPointCloudでpublish
# ※topicはRealSenseに対応しています，アイシンrosbagに適用する場合にはtopic変更の必要あり
# 2022.07.19
# フィルタのパターンを変更 
# 2022.07.18 
# フィルタ：メディアン・バイラテラル・（膨張・縮小？）
# 2値化・HSV領域・輪郭抽出(Canny法)・Hough変換による白線検出方法の検証
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


################################################################################
#                             Grobal Definition                                #
################################################################################
# Binarization range -----------------------------------------------------------
BINARY_LOW	= 190
BINARY_HIGH	= 255

# HSV range --------------------------------------------------------------------
H_LOW	= 0
H_HIGH	= 18
S_LOW	= 13
S_HIGH	= 79
V_LOW	= 142
V_HIGH	= 226

# Canny param ------------------------------------------------------------------
CANNY_RGB_TH1	= 50
CANNY_RGB_TH2	= 130
CANNY_HSV_TH1	= 500
CANNY_HSV_TH2	= 1000

# Median filter ----------------------------------------------------------------
MEDIAN_RGB_SIZE	= 9
MEDIAN_HSV_SIZE	= 3


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
		self.bridge			= CvBridge()

		# Binary range ----------------------
		# self.binary_low		= rospy.get_param('~binary_low', BINARY_LOW)
		# self.binary_high	= rospy.get_param('~binary_high', BINARY_HIGH)

		# HSV range -------------------------
		self.h_low			= rospy.get_param('~h_low',  H_LOW)
		self.h_high			= rospy.get_param('~h_high', H_HIGH)
		self.s_low			= rospy.get_param('~s_low',  S_LOW)
		self.s_high			= rospy.get_param('~s_high', S_HIGH)
		self.v_low			= rospy.get_param('~v_low',  V_LOW)
		self.v_high			= rospy.get_param('~v_high', V_HIGH)
		self.hsv_low		= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
		self.hsv_high		= np.array([self.h_high, self.s_high, self.v_high], np.uint8)

		# # Canny threshold -------------------
		# self.canny_rgb_th1	= rospy.get_param('~canny_rgb_th1',  CANNY_RGB_TH1)
		# self.canny_rgb_th2	= rospy.get_param('~canny_rgb_th2',  CANNY_RGB_TH2)
		# self.canny_hsv_th1	= rospy.get_param('~canny_hsv_th1',  CANNY_HSV_TH1)
		# self.canny_hsv_th2	= rospy.get_param('~canny_hsv_th2',  CANNY_HSV_TH2)

		# # Median filter size ----------------
		self.median_rgb_size	= rospy.get_param('~median_rgb_size', MEDIAN_RGB_SIZE)
		self.median_hsv_size	= rospy.get_param('~median_hsv_size', MEDIAN_HSV_SIZE)


		# CV window for GUI ---------------------------------------------------
		cv2.namedWindow('Parameter Setting')

		# cv2.createTrackbar('Binary low',	'Parameter Setting', self.binary_low,	255, self.paramSetting)
		# cv2.createTrackbar('Binary high',	'Parameter Setting', self.binary_high,	255, self.paramSetting)

		cv2.createTrackbar('H low',			'Parameter Setting', self.h_low,		255, self.paramSetting)
		cv2.createTrackbar('H high',		'Parameter Setting', self.h_high,		255, self.paramSetting)
		cv2.createTrackbar('S low',			'Parameter Setting', self.s_low,		255, self.paramSetting)
		cv2.createTrackbar('S high',		'Parameter Setting', self.s_high,		255, self.paramSetting)
		cv2.createTrackbar('V low',			'Parameter Setting', self.v_low,		255, self.paramSetting)
		cv2.createTrackbar('V high',		'Parameter Setting', self.v_high,		255, self.paramSetting)

		# cv2.createTrackbar('Canny RGB th1',	'Parameter Setting', self.canny_rgb_th1,	300, self.paramSetting)
		# cv2.createTrackbar('Canny RGB th2',	'Parameter Setting', self.canny_rgb_th2,	300, self.paramSetting)
		# cv2.createTrackbar('Canny HSV th1',	'Parameter Setting', self.canny_hsv_th1,	1500, self.paramSetting)
		# cv2.createTrackbar('Canny HSV th2',	'Parameter Setting', self.canny_hsv_th2,	1500, self.paramSetting)

		cv2.createTrackbar('Median RGB size',	'Parameter Setting', self.median_rgb_size,	15, self.paramSetting)
		cv2.createTrackbar('Median HSV size',	'Parameter Setting', self.median_hsv_size,	11, self.paramSetting)


		# Homograpy Matrix (DHAパラメータ)22.5deg
		the_src_pts		= np.array([(122.0, 33.0), (555.0, 23.0), (98.0, 353.0), (572.0, 355.0)], dtype=np.float32)
		the_dst_pts		= np.array([(2.5, 1.0), (2.5, -1.0), (0.5, 0.15), (0.5, -0.15)], dtype=np.float32)
		self.H		= cv2.getPerspectiveTransform(the_src_pts, the_dst_pts)
		# print("H=", self.H)


		# Subscriber -----------------------------------------------------------
		self.front_rgb_image_sub	= rospy.Subscriber('/front_camera/image', Image, self.subFrontRGBImage)		# Front RGB Image
		self.front_rgb_image_sub	= rospy.Subscriber('/front_realsense/color/image_raw', Image, self.subFrontRGBImage)
		# webカメラの場合
		# self.front_rgb_image_sub	= rospy.Subscriber('/head_camera/image_raw', Image, self.subFrontRGBImage)
#		self.rear_rgb_image_sub		= rospy.Subscriber('/rear_camera/image', Image, self.subRearRGBImage)		# Rear RGB Image


		# Publisher ------------------------------------------------------------
		# self.front_binary_image_pub		= rospy.Publisher('/front_camera/binary_image', Image, queue_size=1)		# Front Binary Image

		self.front_hsv_image_pub		= rospy.Publisher('/front_camera/hsv_image', Image, queue_size=1)			# Front HSV Image
		self.front_hsv_mask_image_pub	= rospy.Publisher('/front_camera/hsv_mask_image', Image, queue_size=1)		# Front HSV Mask Image
		# self.front_rgb_canny_image_pub	= rospy.Publisher('/front_camera/rgb_canny_image', Image, queue_size=1)		# Front RGB Canny Image
		# self.front_hsv_canny_image_pub	= rospy.Publisher('/front_camera/hsv_canny_image', Image, queue_size=1)		# Front HSV Canny Image
		# self.front_hsv_mask_canny_image_pub	= rospy.Publisher('/front_camera/hsv_mask_canny_image', Image, queue_size=1)		# Front HSV Mask Canny Image
		# self.front_rgb_median_image_pub				= rospy.Publisher('/front_camera/rgb_median_image', Image, queue_size=1)	# Front RGB Median Image
		self.front_hsv_median_image_pub				= rospy.Publisher('/front_camera/hsv_median_image', Image, queue_size=1)	# Front HSV Median Image(after RGB median)
		self.front_hsv_median_mask_image_pub		= rospy.Publisher('/front_camera/hsv_median_mask_image', Image, queue_size=1)	# Front HSV Median Mask Image(after RGB median)
		# self.front_hsv_median_mask_hough_image_pub	= rospy.Publisher('/front_camera/hsv_median_mask_hough_image', Image, queue_size=1)	# Front HSV Median Mask Hough Image(after RGB median)
#		self.rear_hsv_image_pub		= rospy.Publisher('/rear_camera/hsv_image', Image, queue_size=1)				# Rear HSV Image
#		self.rear_mask_image_pub	= rospy.Publisher('/rear_camera/mask_image', Image, queue_size=1)				# Rear MASK Image

		self.line_points_pub				= rospy.Publisher('/front_camera/line_points', PointCloud, queue_size=1)			# Line Points

		# Timer func ------------------------------------------------------------
		rospy.Timer(rospy.Duration(0.05), self.guiLoop)
		# rospy.Timer(rospy.Duration)

		rospy.spin()
	#===========================================================================
	#   HSV を
	#===========================================================================

	#===========================================================================
	#   GUI Timer Function
	#===========================================================================
	def guiLoop(self, in_dummy):

		the_dumy_img	= np.zeros((10,512,3), np.uint8)
		cv2.imshow('Parameter Setting', the_dumy_img)
		cv2.waitKey(1)


	#===========================================================================
	#   Parameter Setting
	#===========================================================================
	def paramSetting(self, in_dummy):

		self.binary_low		= cv2.getTrackbarPos('Binary low',	'Parameter Setting')
		self.binary_high	= cv2.getTrackbarPos('Binary high',	'Parameter Setting')

		self.h_low			= cv2.getTrackbarPos('H low',		'Parameter Setting')
		self.h_high			= cv2.getTrackbarPos('H high',		'Parameter Setting')
		self.s_low			= cv2.getTrackbarPos('S low',		'Parameter Setting')
		self.s_high			= cv2.getTrackbarPos('S high',		'Parameter Setting')
		self.v_low			= cv2.getTrackbarPos('V low',		'Parameter Setting')
		self.v_high			= cv2.getTrackbarPos('V high',		'Parameter Setting')
		self.hsv_low		= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
		self.hsv_high		= np.array([self.h_high, self.s_high, self.v_high], np.uint8)

		# self.canny_rgb_th1	= cv2.getTrackbarPos('Canny RGB th1',	'Parameter Setting')
		# self.canny_rgb_th2	= cv2.getTrackbarPos('Canny RGB th2',	'Parameter Setting')
		# self.canny_hsv_th1	= cv2.getTrackbarPos('Canny HSV th1',	'Parameter Setting')
		# self.canny_hsv_th2	= cv2.getTrackbarPos('Canny HSV th2',	'Parameter Setting')

		the_median_rgb_size	= cv2.getTrackbarPos('Median RGB size',	'Parameter Setting')
		if (the_median_rgb_size % 2) == 1:
			self.median_rgb_size	= the_median_rgb_size
		else:
			self.median_rgb_size	= the_median_rgb_size + 1
			cv2.createTrackbar('Median RGB size',	'Parameter Setting', self.median_rgb_size,	15, self.paramSetting)

		the_median_size		= cv2.getTrackbarPos('Median HSV size',	'Parameter Setting')
		if (the_median_size % 2) == 1:
			self.median_hsv_size	= the_median_size
		else:
			self.median_hsv_size	= the_median_size + 1
			cv2.createTrackbar('Median HSV size',	'Parameter Setting', self.median_hsv_size,	11, self.paramSetting)

		# print ('\nbinary =', self.binary_low, self.binary_high)
		print ('hsv_low =', self.h_low, self.s_low, self.v_low)
		# print ('hsv_high =', self.h_high, self.s_high, self.v_high)
		# print ('canny_rgb =', self.canny_rgb_th1, self.canny_rgb_th2)
		# print ('canny_hsv =', self.canny_hsv_th1, self.canny_hsv_th2)
		print ('median_size =', self.median_rgb_size, self.median_hsv_size)



	#===========================================================================
	#   Get Front RGB Image
	#===========================================================================
	def subFrontRGBImage(self, in_rgb):

		# # color filtering or transform -----------------------------------------
		self.front_rgb_img		= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')

		# # rgb to binary (binary mask)
		# self.front_gray_img		= cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2GRAY)
		# the_ret, self.front_binary_img	= cv2.threshold(self.front_gray_img, self.binary_low, self.binary_high, cv2.THRESH_BINARY)

		# # rgb to hsv
		self.front_hsv_img		= cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2HSV)

		# # hsv mask with range
		self.front_hsv_mask_img	= cv2.inRange(self.front_hsv_img, self.hsv_low, self.hsv_high)                   

		# # rgb canny contour detection
		# self.front_rgb_canny_img	= cv2.Canny(self.front_rgb_img, self.canny_rgb_th1, self.canny_rgb_th2)

		# # hsv canny contour detection
		# self.front_hsv_canny_img	= cv2.Canny(self.front_hsv_img, self.canny_hsv_th1, self.canny_hsv_th2)

		# # hsv mask canny contour detection
		# self.front_hsv_mask_canny_img	= cv2.Canny(self.front_hsv_mask_img, self.canny_hsv_th1, self.canny_hsv_th2)

		# # rgb median
		self.front_rgb_median_img	= cv2.medianBlur(self.front_rgb_img, self.median_rgb_size)

		# # hsv median
		self.front_hsv_median_img	= cv2.cvtColor(self.front_rgb_median_img, cv2.COLOR_BGR2HSV)

		# hsv median mask with range
		self.front_hsv_median_mask_img	= cv2.inRange(self.front_hsv_median_img, self.hsv_low, self.hsv_high)

		# ホモグラフィ変換
		# http://opencv.jp/opencv-2.1/cpp/geometric_image_transformations.html#warpPerspective
		the_color_h		= self.front_hsv_median_mask_img.shape[0] #color画像の高さ360
		the_color_w		= self.front_hsv_median_mask_img.shape[1] #color画像の横640
		the_points		= PointCloud()
		the_points.header.frame_id	= 'front_realsense_link'
		# print(self.front_hsv_median_mask_img)
		for the_v in range(0, the_color_h, 2):
			for the_u in range(0, the_color_w, 4):
				if self.front_hsv_median_mask_img[the_v, the_u] > 250:
					the_p		= Point32()
					the_p.x		= (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
					the_p.y		= (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
					the_p.z		= 0.0
					the_points.points.append(the_p)
		# print(the_points.points) #
		# hsv median mask hough transform 
		the_lines	= cv2.HoughLinesP(self.front_hsv_median_mask_img, rho=1, theta=np.pi/360.0, threshold=100, minLineLength=40, maxLineGap=15)
		self.front_hsv_median_mask_hough_img = cv2.cvtColor(self.front_hsv_median_mask_img, cv2.COLOR_GRAY2BGR)
		if the_lines is not None:
			print ('line_num =', len(the_lines))
			for x1, y1, x2, y2 in the_lines.squeeze(axis=1):
				cv2.line(self.front_hsv_median_mask_hough_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
		else:
			print ('line_num = 0')

		# # publish images ------------------------------------------------------
		# # binary image (binary mask image)
		# # the_binary_msg		= self.bridge.cv2_to_imgmsg(self.front_binary_img, encoding="mono8")
		# # self.front_binary_image_pub.publish(the_binary_msg)

		# # # hsv image
		the_hsv_msg			= self.bridge.cv2_to_imgmsg(self.front_hsv_img, encoding="rgb8")
		self.front_hsv_image_pub.publish(the_hsv_msg)

		# # # hsv mask image
		the_hsv_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_mask_img, encoding="mono8")
		self.front_hsv_mask_image_pub.publish(the_hsv_mask_msg)

		# # rgb canny image
		# # the_rgb_canny_msg	= self.bridge.cv2_to_imgmsg(self.front_rgb_canny_img, encoding="mono8")
		# # self.front_rgb_canny_image_pub.publish(the_rgb_canny_msg)

		# # hsv canny image
		# # the_hsv_canny_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_canny_img, encoding="mono8")
		# # self.front_hsv_canny_image_pub.publish(the_hsv_canny_msg)

		# # hsv mask canny image
		# the_hsv_mask_canny_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_mask_canny_img, encoding="mono8")
		# self.front_hsv_mask_canny_image_pub.publish(the_hsv_mask_canny_msg)

		# # rgb median image
		the_rgb_median_msg	= self.bridge.cv2_to_imgmsg(self.front_rgb_median_img, encoding="bgr8")
		# self.front_rgb_median_image_pub.publish(the_rgb_median_msg)

		# # # hsv median image
		the_hsv_median_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_median_img, encoding="rgb8")
		self.front_hsv_median_image_pub.publish(the_hsv_median_msg)

		# # hsv median mask image
		the_hsv_median_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_median_mask_img, encoding="mono8")
		self.front_hsv_median_mask_image_pub.publish(the_hsv_median_mask_msg)

		# # hsv median mask hough image
		# the_hsv_median_mask_hough_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_median_mask_hough_img, encoding="rgb8")
		# # self.front_hsv_median_mask_hough_image_pub.publish(the_hsv_median_mask_hough_msg)

		# white line point publish
		self.line_points_pub.publish(the_points)


	#===========================================================================
	#   Get Rear RGB Image
	#===========================================================================
'''
	def subRearRGBImage(self, in_rgb):

		self.rear_rgb_img	= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')

		# rgb to hsv
		self.rear_hsv_img	= cv2.cvtColor(self.rear_rgb_img, cv2.COLOR_BGR2HSV)

		# mask line area
		self.rear_mask_img	= cv2.inRange(self.rear_hsv_img, self.hsv_low, self.hsv_high)

		# publish hsv mask images
		the_hsv_msg		= self.bridge.cv2_to_imgmsg(self.rear_hsv_img, encoding="rgb8")
		self.rear_hsv_image_pub.publish(the_hsv_msg)

		the_mask_msg	= self.bridge.cv2_to_imgmsg(self.rear_mask_img, encoding="mono8")
		self.rear_mask_image_pub.publish(the_mask_msg)
'''


################################################################################
#                               Main Function                                  #
################################################################################
if __name__ == '__main__':

	start = time.time()

	line_detection	= LineDetection() #メイン実行
	
	
	elapsed = time.time() - start
	print(elapsed)
