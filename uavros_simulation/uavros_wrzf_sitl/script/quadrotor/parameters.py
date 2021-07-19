#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: para_com.py
Date: 2021/7/6 16:16
LastEditors: LI Jinjie
LastEditTime: 2021/7/6 16:16
Description: 存储共享的参数
'''
from digi.xbee.util import utils

# Communication related
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 57600

# Set_And_Get_Parameters to be set
PARAM_NODE_ID = "NI"
PARAM_PAN_ID = "ID"
PARAM_DEST_ADDRESS_H = "DH"
PARAM_DEST_ADDRESS_L = "DL"

# PARAM_VALUE_NODE_ID = "GROUND_STATION"
PARAM_VALUE_PAN_ID = utils.hex_string_to_bytes("7FFF")
PARAM_VALUE_DEST_ADDRESS_H = utils.hex_string_to_bytes("00")
PARAM_VALUE_DEST_ADDRESS_L = utils.hex_string_to_bytes("FFFF")

# Mode
MODE_0 = 0  # 准备
MODE_1 = 1  # 起飞
MODE_2 = 2  # 跟踪
MODE_3 = 3  # 返航
Time = 0  # 总体时间

# Name
UAV_A1_NAME = "uav_a1"
UAV_A2_NAME = "uav_a2"
UAV_AC_NAME = "uav_ac"
UAV_SM_NAME = "uav_sm"
UAV_SS_NAME = "uav_ss"
GROUND_STATION_NAME = "ground_station"

# 飞控
FK = 'fk'