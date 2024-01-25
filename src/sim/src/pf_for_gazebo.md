# ガゼボで自己位置推定を行う方法
ターミナル1
roscore

全ターミナルで時間合わせコマンドをする
rosparam set /use_sim_time true

ターミナル2(白線認識、地図、オドメトリ)
roslaunch sim execute_pf_for_gazebo.launch 

検証したい地図でカメラ側、地図側のトピック名を変える必要がある
execute_pf_for_gazebo.launch 内のmap名を、hihuku, origin,tou(1.5_map)にすれば良い

ターミナル3
rosrun sim particle_filter_propose_for_gazebo.cpp 1.5 5

引数説明：
第一引数は地図名:hihuku, origin, 1.5　から選ぶ
第二引数は再生するrosbagのを何個も採っているため、
現在は1~5の数字を入力する
第一引数_mapのフォルダに第二引数.csvファイルを生成する

ターミナル4
rosbag再生0.5倍とか
再生速度は-r 0.5
rosbag play 05_2024-01-23-05-00-22.bag -r 0.5

