# 静止状態でのデータを記録するとき
つまりロボットが勝手に動くのを防ぎたい時</br>

修正点はbeego.urdf.xacroの中身

①type=fixed にすると車輪が回転しなくなる

```
  <joint name="right_wheel_joint" type="continuous">
    <origin xyz="0.02225 -0.1308 -0.01" rpy="${-M_PI/2} 0 0"/>
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <axis xyz="0 0 1"/>
    <!--<limit effort="1.07" velocity="3.14"/>
    <dynamics damping="0.1"/>-->
  </joint>

  <joint name="left_wheel_joint" type="continuous">
    <origin xyz="0.02225 0.1308 -0.01" rpy="${M_PI/2} 0 0"/>
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <axis xyz="0 0 -1"/>
    <!--<limit effort="1.07" velocity="3.14"/>
    <dynamics damping="0.1"/>-->
  </joint>

```


②摩擦係数(：mu)を0にする

```
  <gazebo reference="base_link">
    <selfCollide>true</selfCollide>
    <!-- <mu1 value="0.0"/>
    <mu2 value="0.0"/> -->
    <mu1 value="0.05"/>
    <mu2 value="0.05"/>
  </gazebo>

  <gazebo reference="left_wheel_link">
    <selfCollide>true</selfCollide>
    <!-- <mu1 value="0.0"/>
    <mu2 value="0.0"/> -->
    <mu1 value="1.0"/>
    <mu2 value="1.0"/>
    <!--<kp value="10000000.0"/>
    <kd value="1.0"/>-->
  </gazebo>

  <gazebo reference="right_wheel_link">
    <selfCollide>true</selfCollide>
    <!-- <mu1 value="0.0"/>
    <mu2 value="0.0"/> -->
    <mu1 value="1.0"/>
    <mu2 value="1.0"/>
    <!--<kp value="10000000.0"/>
    <kd value="1.0"/>-->
  </gazebo>

  <gazebo reference="caster_link">
    <selfCollide>true</selfCollide>
    <!-- <mu1 value="0.0"/>
    <mu2 value="0.0"/> -->
    <mu1 value="1.0"/>
    <mu2 value="1.0"/>
  </gazebo>

```

これらをするとcmd_velがtopicででなくなるので注意

# 逆にpfを回すときはロボットを動かすようにする

逆に動かすときは上記を
type=continuousすると動く
摩擦係数muに数字を与えると動く
