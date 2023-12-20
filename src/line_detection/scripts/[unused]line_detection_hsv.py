#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#############################################################################
#########################	UNIVERSITY OF TSUKUBA	#########################
#############################################################################
################################  Python  ###################################
#############################################################################
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
# HSV range ------------------------------------------------------------------
H_LOW	= 0
H_HIGH	= 20
S_LOW	= 26
S_HIGH	= 73
V_LOW	= 120
V_HIGH	= 255


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


		# HSV range -----------------------------
		self.h_low		= rospy.get_param('~h_low',  H_LOW)
		self.h_high		= rospy.get_param('~h_high', H_HIGH)
		self.s_low		= rospy.get_param('~s_low',  S_LOW)
		self.s_high		= rospy.get_param('~s_high', S_HIGH)
		self.v_low		= rospy.get_param('~v_low',  V_LOW)
		self.v_high		= rospy.get_param('~v_high', V_HIGH)
		self.hsv_low	= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
		self.hsv_high	= np.array([self.h_high, self.s_high, self.v_high], np.uint8)

		# 鳥瞰図変換用の座標------------------------
		# 変換前の座標
		self.src_pts = np.array([(240, 80), (420, 80), (40, 180), (620, 180)], dtype=np.float32)
		# 変換先の座標
		self.dst_pts = np.array([(0, 0), (500, 0), (0, 500), (500, 500)], dtype=np.float32)
		# src_pts:変換前の4点の座標, dst_pts: 変換後の座標
		self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)


		# Subscriber -----------------------------------------------------------
		self.front_rgb_image_sub	= rospy.Subscriber('/front_camera/image', Image, self.subFrontRGBImage)		# Front RGB Image
		self.rear_rgb_image_sub		= rospy.Subscriber('/rear_camera/image', Image, self.subRearRGBImage)		# Rear RGB Image


		# Publisher ------------------------------------------------------------
		self.front_hsv_image_pub		   = rospy.Publisher('/front_camera/hsv_image', Image, queue_size=1)			# Front HSV Image
		self.front_mask_image_pub		   = rospy.Publisher('/front_camera/mask_image', Image, queue_size=1)			# Front MASK Image
		self.front_tyoukanzu_image_pub     = rospy.Publisher('/front_camera/tyoukanzu_image', Image, queue_size=1) 
		self.rear_hsv_image_pub			   = rospy.Publisher('/rear_camera/hsv_image', Image, queue_size=1)			# Rear HSV Image
		self.rear_mask_image_pub		   = rospy.Publisher('/rear_camera/mask_image', Image, queue_size=1)			# Rear MASK Image
		self.rear_tyoukanzu_image_pub      = rospy.Publisher('/rear_camera/tyoukanzu_image', Image, queue_size=1)
		
		rospy.spin()


	#===========================================================================
	#   Get Front RGB Image
	#===========================================================================
	def subFrontRGBImage(self, in_rgb):

		self.front_rgb_img	= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')

		# rgb to hsv
		self.front_hsv_img	= cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2HSV)

		# mask line area
		self.front_mask_img	= cv2.inRange(self.front_hsv_img, self.hsv_low, self.hsv_high)

		# 鳥瞰図変換(rgbに対して)
		self.front_tyoukanzu_rgb_img = cv2.warpPerspective(self.front_rgb_img, self.M, (500, 500))

		# publish hsv mask images
		the_hsv_msg		= self.bridge.cv2_to_imgmsg(self.front_hsv_img, encoding="rgb8")
		self.front_hsv_image_pub.publish(the_hsv_msg)

		the_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_mask_img, encoding="mono8")
		self.front_mask_image_pub.publish(the_mask_msg)

		# publish 鳥瞰図変換後の画像(rgb)
		the_toukanz_msg = self.bridge.cv2_to_imgmsg(self.front_tyoukanzu_rgb_img, encoding="rgb8")
		self.front_tyoukanzu_image_pub.publish(the_toukanz_msg)

	#===========================================================================
	#   Get Rear RGB Image
	#===========================================================================
	def subRearRGBImage(self, in_rgb):

		self.rear_rgb_img	= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')

		# rgb to hsv
		self.rear_hsv_img	= cv2.cvtColor(self.rear_rgb_img, cv2.COLOR_BGR2HSV)

		# mask line area
		self.rear_mask_img	= cv2.inRange(self.rear_hsv_img, self.hsv_low, self.hsv_high)
		
		# 鳥瞰図変換(rgbに対して)
		self.rear_tyoukanzu_rgb_img = cv2.warpPerspective(self.rear_rgb_img, self.M, (500, 500))

		# publish hsv mask images
		the_hsv_msg		= self.bridge.cv2_to_imgmsg(self.rear_hsv_img, encoding="rgb8")
		self.rear_hsv_image_pub.publish(the_hsv_msg)

		the_mask_msg	= self.bridge.cv2_to_imgmsg(self.rear_mask_img, encoding="mono8")
		self.rear_mask_image_pub.publish(the_mask_msg)

		# publish 鳥瞰図変換後の画像(rgb)
		the_toukanz_msg = self.bridge.cv2_to_imgmsg(self.rear_tyoukanzu_rgb_img, encoding="rgb8")
		self.rear_tyoukanzu_image_pub.publish(the_toukanz_msg)



################################################################################
#                               Main Function                                  #
################################################################################
if __name__ == '__main__':

	line_detection	= LineDetection()