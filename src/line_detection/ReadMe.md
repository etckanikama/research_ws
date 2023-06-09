# 2022.07.17
## HSV空間での白線領域のMask
### HSV閾値の調整方法
- 実行コマンド
	 > rosrun line_detection check_hsv_range.py
- rosbagから出力した画像を選択
- H,S,Vのバーを動かして白線の領域が白で残るように調整
- 変更時の各値(h_low, s_low, v_low, h_high, s_high, v_high)はターミナルに表示されているので，最終的に良さそうな値を決定

### 実際にリアルタイム処理
- 実行コマンド
	> rosrun line_detection line_detection_hsv.py
- 表示
	> rviz
	- open config > package内のrvizディレクトリ > line_detection.rviz選択
- rosbag play
	> rosbag play 対象のファイル.bag

## 参考サイト
- http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_gui/py_trackbar/py_trackbar.html#trackbar

- https://teratail.com/questions/176114

- https://blog.electroica.com/hsv-trackbar-opencv-python/

- https://pystyle.info/opencv-mask-image/

- https://epic-life.me/archives/1363