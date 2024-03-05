#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties, SpawnModelRequest
from geometry_msgs.msg import Pose
import random
import math
import tf


def get_model_list(model_name='pole_'):
    """
    名前にmodel_nameという文字列が含まれるモデルのリストを取得する関数
    """
    rospy.loginfo('Waiting for service \'gazebo/get_world_properties\'...')
    rospy.wait_for_service('gazebo/get_world_properties')
    try:
        service = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        response = service()
        object_list = response.model_names  # worldに含まれる全てのオブジェクト名
        model_list = [s for s in object_list if model_name in s]  # model_nameを含む要素のみ抽出
        return model_list
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def delete_model(model_name='pole_0'):
    """
    フィールド上のモデルを削除する関数
    """
    rospy.loginfo('Waiting for service \'gazebo/delete_model\'...')
    rospy.loginfo('Model name: ' + model_name)
    rospy.wait_for_service('gazebo/delete_model')
    try:
        service = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        response = service(model_name)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def spawn_sdf_model(pose, sdf_path, model_name='pole_0'):
    """
    フィールドにモデル（.sdf）を出現させる関数
    """
    f = open(sdf_path, 'r')
    xml_string = f.read()
    rospy.loginfo('Waiting for service \/gazebo/spawn_sdf_model\/...')
    rospy.loginfo('Model name: ' + model_name)
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    #print(xml_string)
    try:
        service = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        response = service(model_name, xml_string, "ns", pose, "world")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def spawn_urdf_model(pose, robot_description='/beego/robot_description', model_name='beego'):
    """
    フィールドにモデル（.urdf）を出現させる関数
    """
    xml_string = rospy.get_param(robot_description)
    rospy.loginfo('Waiting for service \/gazebo/spawn_urdf_model\/...')
    rospy.loginfo('Model name: ' + model_name)
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    try:
        service = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        msg = SpawnModelRequest()
        msg.model_name = model_name
        msg.model_xml = xml_string
        msg.robot_namespace = "beego"
        msg.initial_pose = pose
        msg.reference_frame = "world"
        response = service(msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


class PoleInitializer():
    def __init__(self, area='rectangle'):
        self.sdf_path = rospkg.RosPack().get_path('yamasemi_sim')
        self.sdf_path += "/world/pole/model.sdf"
        pole_num_max = rospy.get_param('~pole_num_max', 3)  # ランダム時の最大本数
        pole_num = random.randint(0, int(pole_num_max))
        self.pole_num = rospy.get_param('~pole_num', pole_num)  # 本数（rosparamがあれば優先）
        self.pole_radius = 0.0565  # ポールの半径 [m]
        self.separation = rospy.get_param('~pole_separation', 0.3)  # ポール間（中心距離）を最低どれだけ離すか [m]
        self.search_area = rospy.get_param('~search_area', "rectangle")  # 探索エリアの形状
                                                                         # "rectangle" or "circle"
        if self.search_area == "rectangle":
            self.area_start = tuple(rospy.get_param('~area_start', [0.0, 0.0]))  # 始点 (x1, y1) [m]
            self.area_end = tuple(rospy.get_param('~area_end', [1.0, 1.0]))    # 終点 (x2, y2) [m]
            short_side = min(self.area_end[0]-self.area_start[0], self.area_end[1]-self.area_start[1])
            if self.separation > short_side*0.5:
                # self.separationが大きすぎる場合，収まる程度にする
                self.separation = short_side*0.5
        elif self.search_area == "circle":
            self.area_center = rospy.get_param('~area_center', (0.0, 0.0))  # 中心 (x, y) [m]
            self.area_radius = rospy.get_param('~area_radius', 0.75)  # 半径 [m]
            if self.separation > self.area_radius*0.5:
                # self.separationが大きすぎる場合，収まる程度にする
                self.separation = self.area_radius*0.5

        self.distance = lambda x1, y1, x2, y2: math.sqrt((x2-x1)**2+(y2-y1)**2)

    def generate_rectangle_random_pos(self, start=(0.0, 0.0), end=(1.0, 1.0)):
        """
        探索エリア（四角）の内部にランダムな座標を1点生成する関数
        """
        x_min = self.area_start[0] + self.pole_radius
        x_max = self.area_end[0] - self.pole_radius
        y_min = self.area_start[1] + self.pole_radius
        y_max = self.area_end[1] - self.pole_radius
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        return x, y

    def generate_circle_random_pos(self):
        """
        探索エリア（円）の内部にランダムな座標を1点生成する関数
        """
        r = math.sqrt(random.random()) * (self.area_radius - self.pole_radius)
        theta = random.uniform(-math.pi, math.pi)
        x = self.area_center[0] + r * math.cos(theta)
        y = self.area_center[1] + r * math.sin(theta)
        return x, y

    def delete_and_spawn(self):
        """
        worldにあるポールを削除して再配置する関数
        """
        # フィールドにある全てのポール名を取得
        pole_list = get_model_list(model_name='pole_')

        # ポールを全て削除
        for pole in pole_list:
            delete_model(model_name=pole)

        # ポールをランダムな位置に生成
        x_his, y_his = [], []
        for i in range(self.pole_num):
            while True:
                if self.search_area == "rectangle":
                    x, y = self.generate_rectangle_random_pos()
                elif self.search_area == "circle":
                    x, y = self.generate_circle_random_pos()
                for j in range(i):
                    dist = self.distance(x_his[j], y_his[j], x, y)
                    if dist < self.separation:
                        break  # 座標の生成をやり直し
                else:
                    break  # 座標の生成を完了
            initial_pose = Pose()
            initial_pose.position.x, initial_pose.position.y = x, y
            model_name = 'pole_' + str(i)
            spawn_sdf_model(initial_pose, self.sdf_path, model_name=model_name)
            x_his.append(x); y_his.append(y)


class GateInitializer():
    def __init__(self):
        self.sdf_path = rospkg.RosPack().get_path('yamasemi_sim')
        self.sdf_path += "/world/board/model.sdf"
        self.gate_name = rospy.get_param('~gate_name', 'board_0')
        self.pattern = rospy.get_param('~pattern', 'random')  # 'random', 'A' or 'B'
        if self.pattern == 'random':
            self.pattern = random.choice(('A', 'B'))
        if self.pattern == 'A':
            pose = tuple(rospy.get_param('~pose_A', [0.,]*6))
        elif self.pattern == 'B':
            pose = tuple(rospy.get_param('~pose_B', [0.,]*6))
        else:
            pose = (0.,)*6            
        quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
        self.pose = pose[:3] + tuple(quat)        

    def delete_and_spawn(self):
        """
        world上の指定したゲートを削除して再配置する関数
        """
        delete_model(model_name=self.gate_name)

        initial_pose = Pose()
        initial_pose.position.x = self.pose[0]
        initial_pose.position.y = self.pose[1]
        initial_pose.position.z = self.pose[2]
        initial_pose.orientation.x = self.pose[3]
        initial_pose.orientation.y = self.pose[4]
        initial_pose.orientation.z = self.pose[5]
        initial_pose.orientation.w = self.pose[6]
        
        spawn_sdf_model(initial_pose, self.sdf_path, model_name=self.gate_name)
