# -*- coding: UTF-8 -*-
from can import Message as MSG
from struct import *

# SCALE FACTOR
ANG_SCALE =0.01
RATE_SCALE = 0.01
ERR_SCALE = 0.001

# STATE DICT
servo_state_dict = {
    0x00: "CLOSED",
    0x41: "MANUAL",
    0x44: "POSITION",
    0x66: "TRACK"
}

track_state_dict = {
    0x00: "NoTarget",
    0x01: "Tracking",
    0x02: "Missing"
}
# blue for can_h
# yellow for can_l

class SeekerData:
    def __init__(self):
        # 第一帧率数据
        self.servo_state = "NULL"  # 伺服状态
        self.yaw_rate_sp = 0  # 速度环输入航向角速率 deg/s
        self.pitch_rate_sp = 0  # 速度环输入俯仰角速率 deg/s

        # 第二帧数据
        self.yaw = 0
        self.pitch = 0
        self.yaw_rate = 0
        self.pitch_rate = 0

        # 第三帧数据
        self.track_state = "NULL"  # 跟踪器状态
        self.error_yaw = 0  # 航向角偏差
        self.error_pitch = 0  # 俯仰角偏差

    def __str__(self):
        dataStr = """
state:{} {}
    yaw:{}deg {}deg/s err:{}deg
    pitch:{}deg {}deg/s err:{}deg
        """.format(self.servo_state, self.track_state, self.yaw, self.yaw_rate, self.error_yaw, 
				self.pitch, self.pitch_rate, self.error_pitch)
        return dataStr

def byte2int16(low, high):
    byte_arr = bytearray([low,high])
    # "<"" means little-endian
    # "h" means int16
    return unpack('<h',byte_arr)[0] 
    

def DataParse(msg, SData):
    ID = msg.arbitration_id
    if (ID == 0x008c0000):  # 第一帧
        SData.servo_state = servo_state_dict[msg.data[0]]
        SData.yaw_rate_sp = byte2int16(msg.data[2],msg.data[3]) * RATE_SCALE
        SData.pitch_rate_sp = byte2int16(msg.data[4],msg.data[5]) * RATE_SCALE
    elif (ID == 0x008c0001):  # 第二帧
        SData.yaw = byte2int16(msg.data[0],msg.data[1]) * ANG_SCALE
        SData.pitch = byte2int16(msg.data[2],msg.data[3]) * ANG_SCALE
        SData.yaw_rate = byte2int16(msg.data[4],msg.data[5]) * RATE_SCALE
        SData.pitch_rate = byte2int16(msg.data[6],msg.data[7]) * RATE_SCALE
    elif (ID == 0x008c0002):  # 第三帧
        SData.track_state = track_state_dict[msg.data[3]]
        SData.error_yaw = byte2int16(msg.data[4],msg.data[5]) * ERR_SCALE
        SData.error_pitch = byte2int16(msg.data[6],msg.data[7]) * ERR_SCALE


def TrackingOnMsg():
    msg = MSG(
        arbitration_id=0x0cc80000,
        data=[0x06, 0, 0, 0, 0, 0, 0, 0],
        extended_id=True
    )
    return msg


def TrackingOffMsg():
    msg = MSG(
        arbitration_id=0x0cc80000,
        data=[0x03, 0, 0, 0, 0, 0, 0, 0],
        extended_id=True
    )
    return msg
