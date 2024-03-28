# gazebo環境構築
<!-- ## 概要 -->

### インストール
```
sudo apt-get install ros-noetic-gazeobo-ros-controll
sudo apt-get install ros-noetic-ros-control
sudo apt-get install ros-noetic-ros-controllers
```
ターミナルの保存
```
source /opt/ros/noetic/setup.bash
```


### パッケージの移植＆ビルド

research_wsのsimとrealsense_gazebo_pluginを移植  
その後ビルド
```
catkin build
```
### 起動方法
```
roslaunch sim white_lane.launch
```
※guiをtrueにするとgazeboが起動するがrosbagをとるときは処理が重くなるのでrosbagをとるときはfalseにする（`<arg name="gui" value="true"/>`）


<!-- # Reference
### 白線地図画像をgazebo環境内にinsertする方法
⇛地図環境の画像を差し替えたりするときに使う
⇛集合被覆問題で作成した改良地図や等間隔地図の画像ファイルをgazebo環境に配置するときなどに使用する
⇛worldを保存して、適宜使いたいworldで実行する必要がある







- gazebo_ugokuzo.mdについて(sim/src/urdf/robot_ugokuzo.md)
- install 周りのmdファイルが合ったはず・・・
- 白線地図画像をgazebo環境内にinsertする方法(別途作る？？？)
    - その白線地図画像をpythonで作る方法？？
sim/readme.md起動方法 -->