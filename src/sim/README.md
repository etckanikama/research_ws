# 起動方法
- roslaunch sim white_lane.launch
- gazeboが起動される
- gazeboのinsertを開くwhite_laneを選択

- modelsの中のwhite_laneにモデル画像があるので適宜画像の変更

# シミュレーション上でline_detectionをして任意の場所のパーティクルの尤度の重みを分布させる方法

- sim/launchのwhite_lane.launchの起動
 - ロボットの初期値が明記されている
- rosrun beego_gazebo line_detection_opencv5_homography_timer_for_gazebo.py
で白線点群をpublishするノードの起動
- gazebo上でロボットの座標を変更

gazebo上で変えた座標を以下のプログラムの初期位置に与えて&buildして実行
- likelihood_distribution.cppの起動