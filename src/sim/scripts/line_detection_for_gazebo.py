#!/usr/bin/env python3
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from	sensor_msgs.msg		import PointCloud, PointCloud2,PointField
from	geometry_msgs.msg	import PoseStamped, Point, Quaternion, Point32
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2


# HSV range --------------------------------------------------------------------
H_LOW	= 0
H_HIGH	= 255
S_LOW	= 0
S_HIGH	= 150
V_LOW	= 122
V_HIGH	= 255

class LineDetectionGazebo:
    def __init__(self):
        rospy.init_node("line_detection")
        # rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()
		
        # HSV range ------------------------------------------------------------------
        self.h_low			= rospy.get_param('~h_low',  H_LOW)
        self.h_high			= rospy.get_param('~h_high', H_HIGH)
        self.s_low			= rospy.get_param('~s_low',  S_LOW)
        self.s_high			= rospy.get_param('~s_high', S_HIGH)
        self.v_low			= rospy.get_param('~v_low',  V_LOW)
        self.v_high			= rospy.get_param('~v_high', V_HIGH)
        # バーで調整したhsvのパラメータリストを保存
        self.hsv_low		= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
        self.hsv_high		= np.array([self.h_high, self.s_high, self.v_high], np.uint8)



        # CV  window for GUI-------------------------------------
        cv2.namedWindow("InputImage")
        cv2.setMouseCallback("InputImage", self.mouse_callback)

        cv2.namedWindow('Parameter Setting')
        cv2.createTrackbar('H low',			'Parameter Setting', self.h_low,		255, self.paramSetting)
        cv2.createTrackbar('H high',		'Parameter Setting', self.h_high,		255, self.paramSetting)
        cv2.createTrackbar('S low',			'Parameter Setting', self.s_low,		255, self.paramSetting)
        cv2.createTrackbar('S high',		'Parameter Setting', self.s_high,		255, self.paramSetting)
        cv2.createTrackbar('V low',			'Parameter Setting', self.v_low,		255, self.paramSetting)
        cv2.createTrackbar('V high',		'Parameter Setting', self.v_high,		255, self.paramSetting)
        # --------------------------------------------------------


        # Homograpy 行列の作成：俯角：0.5rad(28.64deg)---------------
        cv_points		= np.array([(9.0, 19.0), (626.0, 17.0), (629.0, 328.0), (10.0, 333.0)], dtype=np.float32)
        ros_points		= np.array([(2.5, 1.7), (2.5, -1.6), (0.5, -0.23), (0.5, 0.24)], dtype=np.float32)
        self.H		= cv2.getPerspectiveTransform(cv_points, ros_points)
        print("H=", self.H)
        # -------------------------------------------------------------



        # subscriber
        self.image_sub = rospy.Subscriber("/beego/my_robo/camera1/image_raw", Image, self.subFrontRGBImage, queue_size=1)
        
        # publisher
        self.image_pub = rospy.Publisher("/output/image_raw", Image, queue_size=1)
        self.front_hsv_mask_image_pub	= rospy.Publisher('/front_camera/hsv_mask_image', Image, queue_size=1)
        self.line_points_pub = rospy.Publisher('/front_camera/line_points', PointCloud, queue_size=1) # Line Points
        self.line_points_downsampling_pub = rospy.Publisher('/front_camera/line_points2',PointCloud2, queue_size=10)
        # ---------------------------------------------------------

        # Spin
        rospy.spin()




    def subFrontRGBImage(self, frame):
        
        # rgb画像を取得
        self.front_rgb_img = self.bridge.imgmsg_to_cv2(frame, 'passthrough')
        self.front_hsv_img		= cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2HSV)

		# H:S:Vのパラメータ調整によってHSV空間の画像に変換
        self.front_hsv_mask_img	= cv2.inRange(self.front_hsv_img, self.hsv_low, self.hsv_high)

        # ホモグラフィ変換
		# http://opencv.jp/opencv-2.1/cpp/geometric_image_transformations.html#warpPerspective
        the_color_h		= self.front_hsv_mask_img.shape[0] #color画像の高さ380(360)
        the_color_w		= self.front_hsv_mask_img.shape[1] #color画像の横640

        the_points		= PointCloud()
        
        the_points.header.frame_id	= 'beego/odom'
        # print("pintsの型:",type(the_points))

        # # ある一点のcv座標をros座標に変換する
        # # cv座標のx
        # the_u = 2.0
        # # cv座標のy
        # the_v = 12.0
        # x1_ros = (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
        # y1_ros =(self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
        # print(x1_ros, y1_ros) #正解は(4.3, 3.1)
        
        the_points_array = []
        for the_v in range(0, the_color_h, 2):
            for the_u in range(0, the_color_w, 4):
                if self.front_hsv_mask_img[the_v, the_u] > 250:
                    x = (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
                    y = (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
                    z = 0.0
                    the_points_array.append([x,y,z])
                    # the_p		= Point32()
                    # the_p.x		= (self.H[0,0]*the_u + self.H[0,1]*the_v + self.H[0,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
                    # the_p.y		= (self.H[1,0]*the_u + self.H[1,1]*the_v + self.H[1,2])/(self.H[2,0]*the_u + self.H[2,1]*the_v + self.H[2,2])
                    # the_p.z		= 0.0
                    # the_points.points.append(the_p)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "beego/odom"
        # フィールド（座標情報）を作成
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        # PointCloud2メッセージを作成
        cloud_msg = pc2.create_cloud_xyz32(header, the_points_array)
        cloud_msg.fields = fields

        self.line_points_downsampling_pub.publish(cloud_msg)

        # チョク
        # print(self.convert_point_cloud(the_points))
		# # print(the_points.points)
		# # hsv median mask hough transform
        # point_cloud2.width = len(the_points.points)

        # # フィールドの設定
        # fields = [
        #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        # ]
        # point_cloud2.fields = fields

        # # ポイントデータの設定
        # point_cloud2.data = []
        # for point in the_points.points:
        #     point_cloud2.data.extend([point.x, point.y, point.z, 0])

        # point_cloud2.point_step = len(point_cloud2.data) // len(the_points.points)
        # point_cloud2.row_step = point_cloud2.point_step * point_cloud2.width
        # print(type(point_cloud2))

        # ____________________________

        the_lines	= cv2.HoughLinesP(self.front_hsv_mask_img, rho=1, theta=np.pi/360.0, threshold=100, minLineLength=40, maxLineGap=15)
        self.front_hsv_median_mask_hough_img = cv2.cvtColor(self.front_hsv_mask_img, cv2.COLOR_GRAY2BGR)
        if the_lines is not None:
            # print ('line_num =', len(the_lines))
            for x1, y1, x2, y2 in the_lines.squeeze(axis=1):
                cv2.line(self.front_hsv_median_mask_hough_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        else:
            print ('line_num = 0')
        
        
        the_hsv_mask_msg	= self.bridge.cv2_to_imgmsg(self.front_hsv_mask_img, encoding="mono8")
        self.front_hsv_mask_image_pub.publish(the_hsv_mask_msg)

		# white line point publish
        self.line_points_pub.publish(the_points)

        the_img = cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2RGB)
        cv2.imshow("InputImage", the_img)
        cv2.waitKey(1)
    


    # Mouseでクリックした座標を取得
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("x: ", x, "y: ", y)


    # def convert_point_cloud(point_cloud):
    #     # Header情報の作成
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = point_cloud.header.frame_id

    #     # PointCloud2メッセージの作成
    #     point_cloud2 = PointCloud2()
    #     point_cloud2.header = header
    #     point_cloud2.height = 1
    #     point_cloud2.width = len(point_cloud.points)

    #     # フィールドの設定
    #     fields = [
    #         PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    #         PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    #         PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
    #     ]
    #     point_cloud2.fields = fields

    #     # ポイントデータの設定
    #     point_cloud2.data = []
    #     for point in point_cloud.points:
    #         point_cloud2.data.extend([point.x, point.y, point.z, 0])

    #     point_cloud2.point_step = len(point_cloud2.data) // len(point_cloud.points)
    #     point_cloud2.row_step = point_cloud2.point_step * point_cloud2.width

    #     return point_cloud2

    # HSVのパラメータcallback
    def paramSetting(self,frame):
        self.h_low			= cv2.getTrackbarPos('H low',		'Parameter Setting')
        self.h_high			= cv2.getTrackbarPos('H high',		'Parameter Setting')
        self.s_low			= cv2.getTrackbarPos('S low',		'Parameter Setting')
        self.s_high			= cv2.getTrackbarPos('S high',		'Parameter Setting')
        self.v_low			= cv2.getTrackbarPos('V low',		'Parameter Setting')
        self.v_high			= cv2.getTrackbarPos('V high',		'Parameter Setting')
        self.hsv_low		= np.array([self.h_low, self.s_low, self.v_low], np.uint8)
        self.hsv_high		= np.array([self.h_high, self.s_high, self.v_high], np.uint8)

        print ('hsv_low =', self.h_low, self.s_low, self.v_low)

if __name__ == '__main__':
    LineDetectionGazebo()
    