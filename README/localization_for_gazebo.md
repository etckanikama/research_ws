# 前提条件
gazebo環境に作成した地図をinsertが完了しており、一度gazebo内をロボットが走行したrosbagがある条件とする
```
cd reserch_ws
souce devel/setup.bush (全ターミナルで通す)
rosparam set /use_sim_time true (全ターミナルで通す)
```
gazebo上で自己位置推定を回す際はフレームのリンクが固定しないようにしてください[参照](gazebo_link_fixed.md)

# 指定の地図環境の呼び出し
launchファイル内のmap_paramを検証したい地図になるように書き換えて実行（被覆地図：hihuku,等間隔地図:tou, 元の地図:origin）
```
roslaunch execute_pf_for_gazebo.launch
```
推定値のフレームで白線点群を可視化出来るようにコード内を以下のように修正  
`header.frame_id = "dha_esti_pose" #gazeboでpf位置推定用`

# 自己位置推定のノード
コマンドライン引数に検証したい地図とどのrosbagを使っているかを書いて実行  
プログラム内部ではコマンドライン引数の値にしたがってcsvで推定値を保存する先のディレクトリ名が変更されるので適切にコマンドライン引数を設定する

```
rosrun sim particle_filter_propose_for_gazebo.cpp origin 1 path2

```
第一引数：検証したい地図(origin, 1.5, hihuku)
第二引数：再生するrosbagの番号（複数回同じ経路でrosbagがあるので）
第三引数：スタートからゴールまでのどの経路のpathを通ったデータか  
path1なら原点(0,0,0)からのものでpath2は逆向きに走行したデータを使用したもの
# rosbagの再生
gazeboで走行したrosbagを0.5倍速で再生
```
rosbag play -r 0.5 01_2024-01-20-09-15-27_three_stacked.bag
```