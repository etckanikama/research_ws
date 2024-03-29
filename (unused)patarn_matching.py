# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# import seaborn as sns

# # 画像の読み込み
# img = cv2.imread('./parking_white_lane_map.png')
# template = cv2.imread('./cropped_image_straight.png')

# # グレースケール化
# img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

# # テンプレートマッチングの実行
# result = cv2.matchTemplate(img_gray, template_gray, cv2.TM_CCOEFF_NORMED)


# # 3Dグラフの作成
# x = np.arange(0, result.shape[1])
# y = np.arange(0, result.shape[0])
# X, Y = np.meshgrid(x, y)
# Z = result

# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # ヒートマップのように、類似度を色で表現
# surf = ax.plot_surface(X, Y, Z, cmap='coolwarm')

# # カラーバーの追加
# fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# # ラベルの追加
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Match Score')

# plt.show()

import cv2
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def match_template_sad(image, template):
    """
    Perform template matching using the Sum of Absolute Differences (SAD) method.

    Parameters:
    image (numpy.ndarray): The source image.
    template (numpy.ndarray): The template image.

    Returns:
    numpy.ndarray: The SAD scores for each position in the source image.
    """
    # Convert images to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    # Get dimensions of the images
    i_height, i_width = image_gray.shape
    t_height, t_width = template_gray.shape

    # Prepare a result matrix
    result = np.zeros((i_height - t_height + 1, i_width - t_width + 1))

    # Perform template matching
    for y in range(result.shape[0]):
        for x in range(result.shape[1]):
            # Extract the current sub-image
            sub_image = image_gray[y:y + t_height, x:x + t_width]

            # Calculate the SAD score
            sad_score = np.sum(np.abs(sub_image.astype(np.float32) - template_gray.astype(np.float32)))
            result[y, x] = sad_score

    return result

# 以下の行は実際の画像のパスに置き換えて実行する必要があります
image = cv2.imread('./parking_white_lane_map.png')
template = cv2.imread('./cropped_image.png')
sad_scores = match_template_sad(image, template)

# ヒートマップの作成と表示
plt.figure(figsize=(10, 8))
sns.heatmap(sad_scores,cmap='coolwarm')
plt.show()