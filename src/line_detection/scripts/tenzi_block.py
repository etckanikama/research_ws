#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessor:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_realsense/color/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/processed/image", Image, queue_size=10)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        processed_img = self.process_image(image)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def process_image(self, image):
        image2 = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        outLineImage = cv2.Canny(gray, 220, 250, apertureSize=3)
        
        houghList = self.hough_lines(image, outLineImage)
        self.draw_cross_points(image, houghList)
        
        houghPList = self.hough_lines_p(image2, outLineImage)
        self.draw_cross_points(image2, houghPList)
        
        return image  # あるいは image2 またはその他の処理結果を返す

    def hough_lines(self, image, outLineImage):
        lineList = []
        lines = cv2.HoughLines(outLineImage, rho=1, theta=np.pi/180, threshold=150) # ハフ変換で直線抽出
        
        # linesがNoneの場合の対応
        if lines is None:
            rospy.loginfo("hough_lines: No lines found")
            return lineList

        rospy.loginfo("hough_lines: %d", len(lines))

        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            lineList.append((x1, y1, x2, y2))

            cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)

        return lineList

    def hough_lines_p(self, image, outLineImage):
        lineList = []
        lines = cv2.HoughLinesP(outLineImage, rho=1, theta=np.pi/180, threshold=200, minLineLength=100, maxLineGap=100)
        
        # linesがNoneの場合の対応
        if lines is None:
            rospy.loginfo("hough_lines_p: No lines found")
            return lineList

        rospy.loginfo("hough_lines_p: %d", len(lines))

        for line in lines:
            x1, y1, x2, y2 = line[0]
            lineList.append((x1, y1, x2, y2))
            cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)

        return lineList

    def draw_cross_points(self, image, lineList):
        size = len(lineList)
        cnt = 0
        for i in range(size-1):
            for j in range(i+1, size):
                pointA = (lineList[i][0], lineList[i][1])
                pointB = (lineList[i][2], lineList[i][3])
                pointC = (lineList[j][0], lineList[j][1])
                pointD = (lineList[j][2], lineList[j][3])
                ret, cross_point = self.calc_cross_point(pointA, pointB, pointC, pointD)
                if ret:
                    if (cross_point[0] >= 0) and (cross_point[0] <= image.shape[1]) and (cross_point[1] >= 0) and (cross_point[1] <= image.shape[0]) :
                        cv2.circle(image, (cross_point[0],cross_point[1]), 2, (255,0,0), 3)
                        cnt = cnt + 1
        rospy.loginfo("draw_cross_points: %d", cnt)

    def calc_cross_point(self, pointA, pointB, pointC, pointD):
        cross_points = (0,0)
        bunbo = (pointB[0] - pointA[0]) * (pointD[1] - pointC[1]) - (pointB[1] - pointA[1]) * (pointD[0] - pointC[0])

        if (bunbo == 0):
            return False, cross_points

        vectorAC = ((pointC[0] - pointA[0]), (pointC[1] - pointA[1]))
        r = ((pointD[1] - pointC[1]) * vectorAC[0] - (pointD[0] - pointC[0]) * vectorAC[1]) / bunbo
        s = ((pointB[1] - pointA[1]) * vectorAC[0] - (pointB[0] - pointA[0]) * vectorAC[1]) / bunbo

        if (r <= 0) or (1 <= r) or (s <= 0) or (1 <= s):
            return False, cross_points

        distance = ((pointB[0] - pointA[0]) * r, (pointB[1] - pointA[1]) * r)
        cross_points = (int(pointA[0] + distance[0]), int(pointA[1] + distance[1]))

        return True, cross_points

def main():
    rospy.init_node('image_processor', anonymous=True)
    ip = ImageProcessor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down image processor node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()