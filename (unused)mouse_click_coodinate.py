import cv2


# マウスで左クリックした座標をプロットしてくれるプログラム
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked coordinates: ({}, {})".format(x, y))
        cv2.putText(image, "({}, {})".format(x, y), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.imshow("Image", image)


# 画像の読み込み
image = cv2.imread("/home/hirayama-d/research_ws/sample_map.png")

# 画像を表示
cv2.imshow("Image", image)

# マウスイベントのコールバック関数を設定
cv2.setMouseCallback("Image", mouse_callback)

# キー入力を待つ
cv2.waitKey(0)

# ウィンドウを閉じる
cv2.destroyAllWindows()
