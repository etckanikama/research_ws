import rospy
import cv2
from	sensor_msgs.msg		import Image, CameraInfo
from cv_bridge	import CvBridge

class TransForm:

    def __init__(self):

        rospy.init_node("transform")

		# openCV ----------------------------
        self.bridge				= CvBridge()
    	
        # Subscriber -----------------------------------------------------------
        self.front_rgb_image_sub	= rospy.Subscriber('/front_realsense/color/image_raw', Image, self.subFrontRGBImage)		# Front RGB Image
		# Publisher ------------------------------------------------------------
        self.front_hsv_image_pub		   = rospy.Publisher('/front_camera/hsv_image', Image, queue_size=1)	

        rospy.spin

    def subFrontRGBImage(self,in_rgb):
        self.front_rgb_img	= self.bridge.imgmsg_to_cv2(in_rgb, 'passthrough')

		# rgb to hsv
        self.front_hsv_img	= cv2.cvtColor(self.front_rgb_img, cv2.COLOR_BGR2HSV)
        the_hsv_msg		= self.bridge.cv2_to_imgmsg(self.front_hsv_img, encoding="rgb8")
        # self.front_hsv_image_pub.publish(the_hsv_msg)
        cv2.imshow(the_hsv_msg)

if __name__ == '__main__':

	transform	= TransForm()

# # ストリーム(Depth/Color)の設定
# config = rs.config()
# #config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# #config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# #
# config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
# #
# config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)

# # ストリーミング開始
# pipeline = rs.pipeline()
# profile = pipeline.start(config)

# # Alignオブジェクト生成
# align_to = rs.stream.color
# align = rs.align(align_to)

# try:
#     while True:

#         # フレーム待ち(Color & Depth)
#         frames = pipeline.wait_for_frames()

#         aligned_frames = align.process(frames)
#         color_frame = aligned_frames.get_color_frame()
#         depth_frame = aligned_frames.get_depth_frame()
#         if not depth_frame or not color_frame:
#             continue

#         #imageをnumpy arrayに
#         color_image = np.asanyarray(color_frame.get_data())
#         depth_image = np.asanyarray(depth_frame.get_data())


#         #depth imageをカラーマップに変換
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

#         #画像表示
#         color_image_s = cv2.resize(color_image, (640, 360))
#         h,w,c = color_image_s.shape
        

#         depth_colormap_s = cv2.resize(depth_colormap, (640, 360))
#         images = np.hstack((color_image_s, depth_colormap_s))
#         # cv2.drawMarker(color_image_s, (320, 180), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
#         # cv2.drawMarker(color_image_s, (197, 57), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
#         # cv2.drawMarker(color_image_s, (456, 51), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
#         # cv2.drawMarker(color_image_s, (75, 185), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
#         # cv2.drawMarker(color_image_s, (603, 185), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
                  
#         # 変換前の座標
#         src_pts = np.array([(197, 57), (456, 51), (75, 185), (603, 185)], dtype=np.float32)
        
#         # 変換後の座標
#         dst_pts = np.array([(0, 0), (500, 0), (0, 500), (500, 500)], dtype=np.float32)

#         # M = cv2.getPerspectiveTransform(src_pts, dst_pts)

#         # # img: 元画像, M: 3x3の変換行列（np型）,出力画像のサイズ（tuple)
#         # dst_img = cv2.warpPerspective(color_image_s, M, (500, 500))



#         cv2.imshow('rgb',color_image_s)
#         # cv2.imshow('dst',dst_img)


#         # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         # cv2.imshow('RealSense', images)

#         if cv2.waitKey(1) & 0xff == 27:#ESCで終了
#             cv2.destroyAllWindows()
#             break

# finally:

#     #ストリーミング停止
#     pipeline.stop()