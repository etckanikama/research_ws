import cv2
import numpy as np

img = cv2.imread("./image0001.png")
print("img_size= ",img.shape)

# 変換前の座標
src_pts = np.array([(240, 80), (420, 80), (40, 180), (620, 180)], dtype=np.float32)
# 変換先の座標
dst_pts = np.array([(0, 0), (500, 0), (0, 500), (500, 500)], dtype=np.float32)
# src_pts:変換前の4点の座標, dst_pts: 変換後の座標
M = cv2.getPerspectiveTransform(src_pts, dst_pts)

# img: 元画像, M: 3x3の変換行列（np型）,出力画像のサイズ（tuple)
dst_img = cv2.warpPerspective(img, M, (500, 500))

cv2.drawMarker(img, (240, 80), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
cv2.drawMarker(img, (420, 80), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
cv2.drawMarker(img, (40, 180), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)
cv2.drawMarker(img, (620, 180), (0,0,255), cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=5)

cv2.imshow('image',img)
cv2.imshow('dst_img',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()