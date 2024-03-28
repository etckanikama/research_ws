# 2024.3.27
# 概要

地図作成用ロボット「さきがけ」を使用した検出した白線を使った、白線地図のグリッドマップ作成と作成した地図を使った自己位置推定までの一貫した処理を行う流れを示す。


# 事前準備  
鳥瞰図変換によるホモグラフィ行列が作成できているものとする   
今回の地図生成用の条件は以下である  
カメラの取付位置：高さ??cm 、前方??cm、下向き角度（俯角）:??度  
以下から、地図作成のための事前準備と流れを示す  
①モバイルバッテリーとvelodyneをつなぐ(type-c)  
②velodyenとハブをpcにつなぐ(usb)  
# 走行データの取得
velodyneとodometryとrealsenseを使って走行データを取得する  
⇛走行コースは地図の原点からスタートしてコース内を周回するようにするようにマニュアル走行する

```
roscore
```
ymbc-sakigake.paramの置いてあるパスを実行する  

```
sudo ypspur-coordinator -p ~/researches/programs/platform/yp-robot-params/robot-params/ymbc-sakigake.param -d /dev/ttyACM0
```
```
rosrun ypspur_ros_bridge ypspur_ros_bridge 
```
④rosbagの保存  

```
roslaunch loclization rosbag_save_for_AMCL_propose.launch
```

# LeGO-loamで3d-lidarの点群地図を保存
④で取得したrosbagを使って点群地図(pcdファイル)を保存  
pcdファイルの作成launchの実行  
```
roslaunch lego_loam run.launch
```
※デフォルトのlego-loamに手を加えたノードにしているのでgithubからそのまま落とすだけでは使えない  
(pcdファイルの出力が出来るようにsrc/mapOptimization.cppとlaunch/run.launchの中身をpcdファイル出力が出来るように書き換えている)

④のrosbagを実行(_2023-11-22-11-13-43_for_point_cloud_map.bag )  
```
rosbag play バグファイル名.bag --clock --topics /velodyne_points
```
完全にrosbag終了したらctr+cでlaunchファイルを止めるLego-loam/data内にpcdファイルが作成される。(ファイル名：PointCloudMap”今日の日付”.pcd")  
※rosbagを途中終了してもpcdファイルは作成されない。


# hdl_localizationの推定値に検出した白線をmap座標系に変換してcsvに保存するまで

leGo-loamによって作成した点群地図を使った自己位置推定を行う  
まずは、上記で作成した点群地図(pcdファイル)をhdl_localization/data内に移動させる  
次に、hdl_localization/launch/hdl_localization.launch内の29行目を自分で名前を付けたpcdファイル(例：sb10_map2.pcd)に書き換える  
```
    <node pkg="nodelet" type="nodelet" name="globalmap_server_nodelet" args="load hdl_localization/GlobalmapServerNodelet $(arg nodelet_manager)">
      <param name="globalmap_pcd" value="$(find hdl_localization)/data/sb10_map2.pcd" />
```
hdl_localizationの実行
```
roslaunch hdl_localization hdl_localization.launch
```

### 推定値をmapフレームを親に持つ新しいフレームでブロードキャスト
hdl_localizationによる推定値(トピック名:/hdl_odom)をmap座標系を親にもった子フレーム(フレーム名：hdl_pose)になるようにブロードキャストする
```
rosrun line_detection paste_pc2_on_3d_map_modify.py
```
### hdl_poseフレームの白線点群情報をpublish
frame_idをhdl_poseにすることで、推定値からみた白線点群（トピック名:/front_camera/line_points_pc2）をpublishするノードの実行
```
rosrun line_detection line_detection_opencv5_homography_timer_aisin_for_rsj.py 
```
このときの鳥瞰図変換のパラメータは地図作成時(rosbagを取った時)でのパラメータにする
※地図作成時に走行したときのカメラ位置でのパラメータにて白線認識から白線点群をpublishする必要がある
```
# # さきがけh:0.91,l=0.41パラメータ：地図作成用
		the_src_pts		= np.array([(5, 8), (632, 4), (5, 355.0), (636.0, 354.0)], dtype=np.float32)
		the_dst_pts		= np.array([(3.3, 2.0), (3.3, -1.8), (1.01, 0.7), (1.01, -0.67)], dtype=np.float32)
		self.H		= cv2.getPerspectiveTransform(the_src_pts, the_dst_pts)
```
照明などで白線の他に白く写ってしまうノイズ処理を行った  
二値化した白線マスク画像から白領域の面積抽出を行い、任意のピクセル集合面積以下の白色情報のノイズをマスクすることでノイズ除去を行った。

### hdl_poseフレームからmapフレームでの白線点群位置に変換して、csvに出力するノード
hdl_poseフレームの点群（トピック名:/front_camera/line_points_pc2）をmapフレームの点群(トピック名：/transformed_pointcloud)に変換して、変換後の白線点群位置を記録CSVファイル(transformed_pointcloud.csv)に出力するノード
```
rosrun line_detection conv_hdl_pose2map.py 
```

### rosbag実行によりmap座標系での白線位置をcsvに記録 
④時に使ったlego-loamのときの地図作成用に走行させたrosbagを回してtransformed_pointcloud.csvを得る  
注意としては実行するrosbagは地図作成用なのでカメラの取り付け位置が地図作成用に取り付けられたときのrosbagを再生する必要がある
```
rosbag play _2023-11-22-11-13-43_for_point_cloud_map.bag --clock 
```
# csvに記録したmap座標系の白線点群位置を貼り合わせた結果から画像の地図に変換する
### csvからpng画像に変換
```
cd line_detection
python3 plot_transformed_point.py
```
### png画像からpgmファイルとyamlファイルを生成してグリッドマップ化
map_serverで画像ファイルを読み込めるようにpng画像をpgmとyamlファイルに変換する必要がある。  
地図原点とサイズ、解像度を指定することでグリッドマップを作成できるプログラム  
```
cd localization
python3 conv_png2pgm_yaml.py
```
白線位置を黒色に、それ以外を走行可能領域としてグレーとした画像を作成する


# gazebo環境における作成したグリッドマップを地図情報として使ったときの自己位置推定
## 事前準備
```
roscore
```
全部のターミナルで
```
rosparam set /use sim time true
```
gazeboで走行したときのrosbagを準備(01_2024-01-20-09-15-27_three_stacked.bag)  

実行するrosbagの初期位置・姿勢に合わせて以下の2つのファイルファイルを修正
- sim/src/gridmap_localization_for_gazebo.cpp内のinit_x,init_y,init_yawの値を変更
- sim/src/beego_odom_publisher.cpp内のinit_x,init_y,init_yawの値を変更
地図原点からスタートするパスのrosbagを回すときは0,0,0にする  
逆順のパスを通るときのrosbagを回すときは 

## gazebo走行データでのグリッドマップでの自己位置推定を行う
### 作成したgridmapの情報をpublishする
```
roslaunch localization show_map.launch
```
/mapトピックでグリッドマップ上でのどのピクセル位置に色を割り当てたかどうかのvalueが入っており、白情報のvalueは100で走行可能領域は0という値が含まれる

### 白線点群のpublishと真値をrviz上に出力するノードをまとめて実行する

```
roslaunch sim  execute_pf_for_gazebo.launch 
```
鳥瞰図変換のパラメータはgazebo内で調整したものを使用している  
この時、map_paramの値は実行したい地図のパラメータにする。今回のrosbagと使用する地図は元の地図を使用するためmap_paramにはoriginを入力する（※header.frame_id = "grid_dha_esti_pose" #gridマップでのpf推定値に設定する）

### gazebo走行データで自己位置推定をする
```
rosrun sim gridmap_localization_for_gazebo.cpp
```
gazebo走行データを再生(処理が遅くなる可能性があるので0.5倍速で再生)
```
rosbag play -r 0.5 01_2024-01-20-09-15-27_three_stacked.bag
```

