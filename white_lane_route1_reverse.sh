#!/bin/sh

source ~/research_ws/devel/setup.bash 

for initial_y in $(seq -3.60 0.1 0.0)
do
    # launchファイルを実行
    roslaunch sim white_lane.launch initial_x:=10.7 initial_y:=$initial_y initial_yaw:=1.57 &
    # プロセスIDを取得
    pid_launch=$!
    sleep 1

    # rosrunでノードを起動
    rosrun sim likelyfood_distribution_for_gazebo.cpp 10.7 $initial_y 1.57 &
    # プロセスIDを取得
    pid_rosrun=$!
    sleep 1

    rosrun sim line_detection_opencv5_homography_timer_for_gazebo.py &
    sleep 1
    pid_rosrun2=$!

    # 数秒待つ（例：15秒）
    sleep 10
    scrot "matching-10.7-${initial_y}-1.57.png" -u -e 'mv $f /home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/output/matching'


    # プロセスを終了
    kill $pid_launch
    kill $pid_rosrun
    kill -9 $pid_rosrun2

    # 更に数秒待つ（例：25秒）
    sleep 25
done


for initial_x in $(seq 10.7 -0.1 0.0)
do
    # launchファイルを実行
    roslaunch sim white_lane.launch initial_x:=$initial_x initial_y:=0.0 initial_yaw:=-3.14 &
    # プロセスIDを取得
    pid_launch=$!
    sleep 1

    # rosrunでノードを起動
    rosrun sim likelyfood_distribution_for_gazebo.cpp $initial_x 0.0 -3.14 &
    # プロセスIDを取得
    pid_rosrun=$!
    sleep 1

    rosrun sim line_detection_opencv5_homography_timer_for_gazebo.py &
    sleep 1
    pid_rosrun2=$!

    # 数秒待つ（例：15秒）
    sleep 10
    scrot "matching-${initial_x}-0.0--3.14.png" -u -e 'mv $f /home/hirayama-d/research_ws/src/sim/20240101_after_set_covering_route1_tougou/path_route1_reverse/output/matching'

    # プロセスを終了
    kill $pid_launch
    kill $pid_rosrun
    kill -9 $pid_rosrun2

    # 更に数秒待つ（例：25秒）
    sleep 25
done