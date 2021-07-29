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
BAUD_RATE = 57600

# Set_And_Get_Parameters to be set
PARAM_NODE_ID = "NI"
PARAM_PAN_ID = "ID"
PARAM_DEST_ADDRESS_H = "DH"
PARAM_DEST_ADDRESS_L = "DL"

# PARAM_VALUE_NODE_ID = "GROUND_STATION"
PARAM_VALUE_PAN_ID = utils.hex_string_to_bytes("5FFF")  # position_sender: 8FFF   decision part: 7FFF
PARAM_VALUE_DEST_ADDRESS_H = utils.hex_string_to_bytes("00")
PARAM_VALUE_DEST_ADDRESS_L = utils.hex_string_to_bytes("FFFF")

# Name
SENDER_NAME = "tx_1"
RECEIVER_1_NAME = "rx_1"
RECEIVER_2_NAME = "rx_2"



