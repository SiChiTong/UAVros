#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: cmd_com.py
Date: 2021/7/8 15:45
LastEditors: LI Jinjie
LastEditTime: 2021/7/8 15:45
Description: 专门存储发送的指令类型
'''

# 按照字典的格式发送  # {"NODE":node name, "CMD": command}
KEY_NODE = "NODE"
KEY_CMD = "CMD"
KEY_POSITION = "POS"

# Command
# 模式切换
TO_MODE_1 = "ch mode 1"
TO_MODE_2 = "ch mode 2"
TO_MODE_3 = "ch mode 3"

# 准备完毕
PREP_SUCCESS = "prep success"
ALL_SUCCESS = "all success"
REACH_SUCCESS = "reach success"  # 表示在跟踪模式中，到达指定位置，完成动作。
FLY = "fly"

# 包含位置的命令
TARGET_POS = "target"  # 在起飞阶段告诉每个飞机飞往的位置
TRIM_POS = "trim"  # 在飞到上空后对每个飞机进行微调
FORMATION_POS = "f"  # 必须同时有位置数据，用于向其他无人机发送位置组成编队

# 数字检测
DETECT_NUM = ["detect 0",
              "detect 1",
              "detect 2",
              "detect 3"]

# 动作
GO_UP = "go up"
GO_DOWN = "go down"
GO_TO_CAR_NUM = ["car 0",
                 "car 1",
                 "car 2"]
GO_CLOSE = 'go close'
GO_FAR = 'go far'

RETURN = 'return'
