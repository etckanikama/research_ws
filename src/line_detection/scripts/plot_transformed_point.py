import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルからデータを読み込む
csv_file_path = '/home/hirayama-d/research_ws/src/line_detection/scripts/transformed_pointcloud.csv'
csv_data = pd.read_csv(csv_file_path)
# 画像のサイズと初期背景の設定
width = 662
height = 1263
image = np.full((height, width, 3), (65, 105, 225), dtype=np.uint8)

# cvから見たROS座標原点(x0, y0)の設定
x0_cv = 301
y0_cv = 1263

# ROS座標からCV座標に変換
cv_data = csv_data.copy()
cv_data['cv_x'] = x0_cv - cv_data['y'] * 100
cv_data['cv_y'] = y0_cv - csv_data['x'] * 100

# 変換したCV座標系のcv_x, cv_yの座標を白で画像を塗りつぶす
for index, row in cv_data.iterrows():
    cv_x, cv_y = int(row['cv_x']), int(row['cv_y'])
    if 0 <= cv_x < width and 0 <= cv_y < height:
        image[cv_y, cv_x] = (255, 255, 255)  # 白で塗りつぶす

# 画像データをBGRからRGBに変換
image_rgb = image[:, :, [2, 1, 0]]

# 画像を表示
plt.imsave('transformed_image.png', image_rgb)
plt.imshow(image_rgb)
plt.show()
