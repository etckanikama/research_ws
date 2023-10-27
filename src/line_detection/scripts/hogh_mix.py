
import numpy as np
import cv2

# IMAGE_PATH = "./test.jpg" # 読み込む画像
IMAGE_PATH = "./kakunin_01.png" # 読み込む画像
def main():
    image  = cv2.imread(IMAGE_PATH) # 画像読み込み
    image2 = cv2.imread(IMAGE_PATH) # 画像読み込み

    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) # グレースケール化
    outLineImage = cv2.Canny(gray, 220, 250, apertureSize = 3)   # 輪郭線抽出
    cv2.imwrite("./outLine.png", outLineImage)    # ファイル保存

    houghList  = hough_lines(image, outLineImage)  # ハフ変換による直線抽出
    cv2.imwrite("./result_hough.png", image)       # ファイル保存
    draw_cross_points(image, houghList)            # 直線リストから交点を描画
    cv2.imwrite("./result_hough_cross.png", image) # ファイル保存

    houghPList = hough_lines_p(image2, outLineImage)  # 確率的ハフ変換による直線抽出
    cv2.imwrite("./result_houghP.png", image2)        # ファイル保存
    draw_cross_points(image2, houghPList)             # 直線リストから交点を描画
    cv2.imwrite("./result_houghP_cross.png", image2)  # ファイル保存


# ハフ変換で直線を抽出する関数
def hough_lines(image, outLineImage):
    lineList = []
    lines = cv2.HoughLines(outLineImage, rho=1, theta=np.pi/180, threshold=200) # ハフ変換で直線抽出
    print("hough_lines: ", len(lines))

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

        cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2) # 赤色で直線を引く

    return lineList


# 確率的ハフ変換で直線を抽出する関数
def hough_lines_p(image, outLineImage):
    lineList = []
    # 確率的ハフ変換で直線を抽出
    lines = cv2.HoughLinesP(outLineImage, rho=1, theta=np.pi/180, threshold=200, minLineLength=100, maxLineGap=100)
    print("hough_lines_p: ", len(lines))

    for line in lines:
        x1, y1, x2, y2 = line[0]
        lineList.append((x1, y1, x2, y2))
        cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2) # 緑色で直線を引く

    return lineList


# 交点を描画する関数
def draw_cross_points(image, lineList):
    size = len(lineList)

    cnt = 0
    for i in range(size-1):
        for j in range(i+1, size):
            pointA = (lineList[i][0], lineList[i][1])
            pointB = (lineList[i][2], lineList[i][3])
            pointC = (lineList[j][0], lineList[j][1])
            pointD = (lineList[j][2], lineList[j][3])
            ret, cross_point = calc_cross_point(pointA, pointB, pointC, pointD) # 交点を計算
            if ret:
                # 交点が取得できた場合でも画像の範囲外のものは除外
                if (cross_point[0] >= 0) and (cross_point[0] <= image.shape[1]) and (cross_point[1] >= 0) and (cross_point[1] <= image.shape[0]) :
                    cv2.circle(image, (cross_point[0],cross_point[1]), 2, (255,0,0), 3) # 交点を青色で描画
                    cnt = cnt + 1
    print("draw_cross_points:", cnt)


# 線分ABと線分CDの交点を求める関数
def calc_cross_point(pointA, pointB, pointC, pointD):
    cross_points = (0,0)
    bunbo = (pointB[0] - pointA[0]) * (pointD[1] - pointC[1]) - (pointB[1] - pointA[1]) * (pointD[0] - pointC[0])

    # 直線が平行な場合
    if (bunbo == 0):
        return False, cross_points

    vectorAC = ((pointC[0] - pointA[0]), (pointC[1] - pointA[1]))
    r = ((pointD[1] - pointC[1]) * vectorAC[0] - (pointD[0] - pointC[0]) * vectorAC[1]) / bunbo
    s = ((pointB[1] - pointA[1]) * vectorAC[0] - (pointB[0] - pointA[0]) * vectorAC[1]) / bunbo

    # 線分AB、線分AC上に存在しない場合
    if (r <= 0) or (1 <= r) or (s <= 0) or (1 <= s):
        return False, cross_points

    # rを使った計算の場合
    distance = ((pointB[0] - pointA[0]) * r, (pointB[1] - pointA[1]) * r)
    cross_points = (int(pointA[0] + distance[0]), int(pointA[1] + distance[1]))

    # sを使った計算の場合
    # distance = ((pointD[0] - pointC[0]) * s, (pointD[1] - pointC[1]) * s)
    # cross_points = (int(pointC[0] + distance[0]), int(pointC[1] + distance[1]))

    return True, cross_points

if __name__ == '__main__':
    main()