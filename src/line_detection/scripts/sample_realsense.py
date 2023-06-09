from email.mime import image
import pyrealsense2 as rs
import numpy as np
import cv2

# ストリーム(Depth/Color)の設定
config = rs.config()
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
#
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)

# ストリーミング開始
pipeline = rs.pipeline()
profile = pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:

        # フレーム待ち(Color & Depth)
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame or not color_frame:
            continue

        #imageをnumpy arrayに
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())


        #depth imageをカラーマップに変換
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        #画像表示
        color_image_s = cv2.resize(color_image, (640, 360))
        h,w,c = color_image_s.shape
        

        depth_colormap_s = cv2.resize(depth_colormap, (640, 360))
        images = np.hstack((color_image_s, depth_colormap_s))
        # cv2.drawMarker(color_image_s, (320, 180), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
        # cv2.drawMarker(color_image_s, (66, 17), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
        # cv2.drawMarker(color_image_s, (548, 9), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
        # cv2.drawMarker(color_image_s, (138, 304), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
        # cv2.drawMarker(color_image_s, (592, 303), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
                  
        # # 変換前の座標
        src_pts = np.array([(66, 17), (548, 9), (138, 304), (592, 303)], dtype=np.float32)
        
        # 変換後の座標
        dst_pts = np.array([(1.5, 0.75), (1.5, -0.5), (0.5, 0.15), (0.5, -0.15)], dtype=np.float32)

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        print(M)
        # # img: 元画像, M: 3x3の変換行列（np型）,出力画像のサイズ（tuple)
        # dst_img = cv2.warpPerspective(color_image_s, H, (500, 500))



        cv2.imshow('rgb',color_image_s)
        # cv2.imshow('dst',dst_img)


        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        if cv2.waitKey(1) & 0xff == 27:#ESCで終了
            cv2.destroyAllWindows()
            break

    # H = np.array([1.24430478e-04 ,1.93009414e-03 ,1.80214859e+00],[-3.21563648e-03, 1.10662522e-04, 1.13193705e+00],[2.67711639e-04, 1.24183981e-02, 1.00000000e+00]])


finally:

    #ストリーミング停止
    pipeline.stop()