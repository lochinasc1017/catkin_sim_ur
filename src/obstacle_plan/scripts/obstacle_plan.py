#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

import os
import numpy as np
import linecache
import re
import shutil

import math


# 旋转矩阵2四元数
def rm2q(rm):
    q = np.zeros(4)
    q[3] = math.sqrt(1+rm[0, 0]+rm[1, 1]+rm[2, 2])/2
    q[0] = (rm[2,1] - rm[1,2])/(4*q[3])
    q[1] = (rm[0,2] - rm[2,0])/(4*q[3])
    q[2] = (rm[1,0] - rm[0,1])/(4*q[3])

    return q

# 固定轴角 2 旋转矩阵(三角函数弧度为单位)
def ax2rm(r, p, y):
    # r_m = np.zeros(3, 3)
    RZ = [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0,1]]
    RY = [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
    RX = [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
    tem = np.dot(RZ, RY)
    result = np.dot(tem, RX)

    return result    


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 等待gazebo和moveit启动
        rospy.sleep(1)
        
        # 初始化ROS节点
        rospy.init_node('pose_init')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
                      
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)

        # current_pose = arm.get_current_pose()
        # print("the current pose is :", current_pose)
        # current_rpy = arm.get_current_rpy()
        # print('the current rpy is : ', current_rpy)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('up')
        arm.go()
        rospy.sleep(0.5)


        # 运动位置设置
        sta_xyz = [-0.454, 0.426, 0.03]
        sta_rpy = [1.567, 1.469, -3.080]
        goal_xyz = [0.454, 0.426, 0.03]
        goal_rpy = [1.567, 1.469, -3.080]

     
                       
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系

        sta_rm = ax2rm(sta_rpy[0], sta_rpy[1], sta_rpy[2])
        sta_quat = rm2q(sta_rm)

        goal_rm = ax2rm(goal_rpy[0], goal_rpy[1], goal_rpy[2])
        goal_quat = rm2q(goal_rm)

        # print('goal_quat is :', goal_quat)

        start_pose = PoseStamped()
        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose.position.x = sta_xyz[0]
        start_pose.pose.position.y = sta_xyz[1]
        start_pose.pose.position.z = sta_xyz[2]

        start_pose.pose.orientation.x = sta_quat[0]
        start_pose.pose.orientation.y = sta_quat[1]
        start_pose.pose.orientation.z = sta_quat[2]
        start_pose.pose.orientation.w = sta_quat[3]


        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(start_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(2)

        # goal plan and execute
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = reference_frame
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = goal_xyz[0]
        goal_pose.pose.position.y = goal_xyz[1]
        goal_pose.pose.position.z = goal_xyz[2]

        goal_pose.pose.orientation.x = goal_quat[0]
        goal_pose.pose.orientation.y = goal_quat[1]
        goal_pose.pose.orientation.z = goal_quat[2]
        goal_pose.pose.orientation.w = goal_quat[3]

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(goal_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()

        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(2)
     
        # 控制机械臂回到初始化位置
        arm.set_named_target('up')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
