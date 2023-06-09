#!/usr/bin/env python3
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
###                           check_hsv_range.py                          ###
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


################################################################################
#                                   Import                                     #
################################################################################
import	rospy
import	cv2
import	copy
import  tkinter
import  tkinter.filedialog as fd
import	numpy as np

#trackbar callback fucntion to update HSV value
def callback(x):
	global h_low, h_high, s_low, s_high, v_low, v_high
	#assign trackbar position value to H,S,V High and low variable
	h_low	= cv2.getTrackbarPos('H low',	'HSV transform')
	h_high	= cv2.getTrackbarPos('H high',	'HSV transform')
	s_low	= cv2.getTrackbarPos('S low',	'HSV transform')
	s_high	= cv2.getTrackbarPos('S high',	'HSV transform')
	v_low	= cv2.getTrackbarPos('V low',	'HSV transform')
	v_high	= cv2.getTrackbarPos('V high',	'HSV transform')

	print ('\nhsv_low =', h_low, s_low, v_low)
	print ('hsv_high =', h_high, s_high, v_high)

# Fileのdialogを呼び出し、Pathを取得する
def get_file_path(stitle):

    file_path = fd.askopenfilename(
        title = stitle
    )
    return file_path

file_path = get_file_path("画像ファイルを選択して下さい")
str_file_path = str(file_path)

# Read camera image, a window
rgb_img		= cv2.imread(file_path)
print ('image_size=', rgb_img.shape[:3])
cv2.namedWindow('HSV transform')

# global variables
h_low	= 0
h_high	= 255
s_low	= 0
s_high	= 255
v_low	= 0
v_high	= 255

# create trackbars for color change
cv2.createTrackbar('H low',	'HSV transform', h_low,  255, callback)
cv2.createTrackbar('H high','HSV transform', h_high, 255, callback)
cv2.createTrackbar('S low',	'HSV transform', s_low,  255, callback)
cv2.createTrackbar('S high','HSV transform', s_high, 255, callback)
cv2.createTrackbar('V low',	'HSV transform', v_low,  255, callback)
cv2.createTrackbar('V high','HSV transform', v_high, 255, callback)




while(1):
	# convert sourece image to HSV color mode
	hsv_img		= cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)

	# making mask for hsv range 2値化
	hsv_low		= np.array([h_low, s_low, v_low], np.uint8)
	hsv_high	= np.array([h_high, s_high, v_high], np.uint8)
	mask_img	= cv2.inRange(hsv_img, hsv_low, hsv_high)
	# 鳥瞰図変換

	# show image
	mask_disp_img = cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)	#そのままだと形式が違って結合できないので変更
	disp_img	= cv2.hconcat([rgb_img, hsv_img, mask_disp_img])
	cv2.imshow('HSV transform', disp_img)

	# waitfor the user to press escape and break the while loop 
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break


#destroys all window
cv2.destroyAllWindows()




