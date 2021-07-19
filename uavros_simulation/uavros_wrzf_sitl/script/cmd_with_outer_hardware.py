#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@Author  : Li-Jinjie
@Time    : 2021/7/13 17:27
@File    : cmd_with_outer_hardware
@Email   : lijinjie362@outlook.com
@Description:  None
'''
# 导引头给决策
FROM_DYT_F = "from_dyt.txt"

DETECT_SUCCESS = "success"

# 决策给导引头
TO_DYT_F = "to_dyt.txt"

DYT_SHUT_DOWN = "shut down"
DYT_OPEN = "open"

# 飞控给决策
FROM_FK_F = "from_fk.txt"

# ARRIVED = "arrived"  # 表示飞到目标区域

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

WAIT = "wait"

FLY = "takeoff"  # 表示起飞

RETURN = "return"

# YOLO给决策
FROM_YOLO_F_1 = "from_yolo_1.txt"
FROM_YOLO_F_2 = "from_yolo_2.txt"

# 发给我数字

# 与飞控进行状态交互
STATE_F_C = "state_c.txt"
STATE_F_1 = "state_1.txt"
STATE_F_2 = "state_2.txt"
