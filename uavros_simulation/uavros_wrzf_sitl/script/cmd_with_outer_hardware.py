#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@Author  : Li-Jinjie
@Time    : 2021/7/13 17:27
@File    : cmd_with_outer_hardware
@Email   : lijinjie362@outlook.com
@Description:  None
'''

# 飞控给决策
FROM_FK_F = "from_fk.txt"

# 飞控给决策的位置文件
FROM_FK_STATE_F = "position.json"

ARRIVED = "arrived"  # 表示飞到目标区域

REACH_SUCCESS = "success"
REACH_FALSE = "failure"

REACH_TO_CAR = ["reach to car 0",
                "reach to car 1",
                "reach to car 2"]  # 表示飞到车的位置
GO_TO_CAR_NUM = ["car 0",
                 "car 1",
                 "car 2"]

# 决策给飞控
TO_FK_F = "jc2fk.txt"

WAIT = "wait"  # 等待模式
FLY = "takeoff"  # 表示起飞
RETURN = "return"

# YOLO给决策
FROM_YOLO_F_1 = "from_yolo_1.txt"
FROM_YOLO_F_2 = "from_yolo_2.txt"

# 发给我数字

# 与飞控进行状态交互
# STATE_F_C = "state_c.txt"
Follower_Action_None = 0
Follower_Action_Land = 1
Follower_Action_RTL = 2
Follower_Action_Takeoff = 3
Follower_Action_Follow = 4
Follower_Action_Arm = 5
Follower_Action_Terminate = 6

# dui xing
TYPE_CLOSE = 1
TYPE_FAR = 2
