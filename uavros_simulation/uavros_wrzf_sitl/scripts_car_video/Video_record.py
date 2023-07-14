#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import cv2
import time
import numpy as np
import math
from cir2 import findcir
import serial
import socket
import codecs

IP = '127.0.0.1'
#IP = '192.168.1.170'

port = 9999
agent_name = 'agent1'

try:
    client_sk = socket.socket()  # 连接服务端
    client_sk.connect((IP, port))
except socket.error as msg:
    print(msg)
    sys.exit(1)

print(str(client_sk.recv(1024)))  # 连接成功，接收server的welcome

while 1:
    data = client_sk.recv(1024)
    datad = data.decode('utf-8')
    if datad == 'begin':
        break

if __name__ == "__main__":
    now = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime(time.time()))
    fname = now + agent_name + r"_VideoRecord.mp4"
    file_txt_name = now + agent_name+r"_DataRecord.txt"
    cap = cv2.VideoCapture(0)  # 捕获摄像头的帧
    cap.set(3, 1280)  # 设置分辨率
    cap.set(4, 720)
    success, frame = cap.read()  # 读取摄像头数据
    flag_gate = 0  # 自设标志位
    Dxy = [0, 0]  # 存储目标中心数据
    num = 0
    dxy_num = 0
    a1 = frame.shape[0]  # 高
    b1 = frame.shape[1]  # 宽
    a2 = 1280
    b2 = 720
    greeen = (0, 255, 0)
    white = (255, 255, 255)
    p = 0.00
    size = (a2,b2)
    fps = 30
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter()
    out.open(fname,fourcc, fps, size)
    with codecs.open(file_txt_name, mode='a', encoding='utf-8') as file_txt:
        while success:

            cv2.line(frame, (int(a2 / 4), int(b2 / 5)), (int(a2 / 4), int(4 * b2 / 5)), greeen, 2)
            cv2.line(frame, (int(3 * a2 / 4), int(b2 / 5)), (int(3 * a2 / 4), int(4 * b2 / 5)), greeen, 2)

            line_point = (
                ((int(a2 / 4) - 30, int(b2 / 5)), (int(a2 / 4), int(b2 / 5))),
                ((int(a2 / 4) - 20, int(b2 * 3 / 10)), (int(a2 / 4), int(b2 * 3 / 10))),
                ((int(a2 / 4) - 20, int(b2 * 4 / 10)), (int(a2 / 4), int(b2 * 4 / 10))),
                ((int(a2 / 4) - 30, int(b2 * 5 / 10)), (int(a2 / 4), int(b2 * 5 / 10))),
                ((int(a2 / 4) - 20, int(b2 * 6 / 10)), (int(a2 / 4), int(b2 * 6 / 10))),
                ((int(a2 / 4) - 20, int(b2 * 7 / 10)), (int(a2 / 4), int(b2 * 7 / 10))),
                ((int(a2 / 4) - 30, int(b2 * 8 / 10)), (int(a2 / 4), int(b2 * 8 / 10))),
                ((int(a2 * 3 / 4), int(b2 / 5)), (int(a2 * 3 / 4) + 30, int(b2 / 5))),
                ((int(a2 * 3 / 4), int(b2 * 3 / 10)), (int(a2 * 3 / 4) + 20, int(b2 * 3 / 10))),
                ((int(a2 * 3 / 4), int(b2 * 4 / 10)), (int(a2 * 3 / 4) + 20, int(b2 * 4 / 10))),
                ((int(a2 * 3 / 4), int(b2 * 5 / 10)), (int(a2 * 3 / 4) + 30, int(b2 * 5 / 10))),
                ((int(a2 * 3 / 4), int(b2 * 6 / 10)), (int(a2 * 3 / 4) + 20, int(b2 * 6 / 10))),
                ((int(a2 * 3 / 4), int(b2 * 7 / 10)), (int(a2 * 3 / 4) + 20, int(b2 * 7 / 10))),
                ((int(a2 * 3 / 4), int(b2 * 8 / 10)), (int(a2 * 3 / 4) + 30, int(b2 * 8 / 10)))
            )
            for i in range(0, 14):
                cv2.line(frame, line_point[i][0], line_point[i][1], greeen, 2)

            font = cv2.FONT_HERSHEY_SIMPLEX
            #cv2.putText(frame, '30', (int(a2 / 4) - 75, int(b2 / 5) + 10), font, 1, greeen, 2)
            #cv2.putText(frame, '-30', (int(a2 / 4) - 100, int(b2 * 4 / 5) + 10), font, 1, greeen, 2)
            #cv2.putText(frame, '0', (int(a2 / 4) - 55, int(b2 / 2) + 10), font, 1, greeen, 2)
            #cv2.putText(frame, '30', (int(a2 * 3 / 4) + 35, int(b2 / 5) + 10), font, 1, greeen, 2)
            #cv2.putText(frame, '-30', (int(a2 * 3 / 4) + 35, int(b2 * 4 / 5) + 10), font, 1, greeen, 2)
            #cv2.putText(frame, '0', (int(a2 * 3 / 4) + 35, int(b2 / 2) + 10), font, 1, greeen, 2)

            cv2.circle(frame, (int(a2 / 2), int(b2 / 2)), 30, white)
            cv2.circle(frame, (int(a2 / 2), int(b2 / 2)), 60, white)
            cv2.line(frame, (int(a2 / 2) - 80, int(b2 / 2)), (int(a2 / 2) + 80, int(b2 / 2)), white, 1)
            cv2.line(frame, (int(a2 / 2), int(b2 / 2) - 80), (int(a2 / 2), int(b2 / 2) + 80), white, 1)
            if Dxy[0] != -35:
                cv2.putText(frame, str(round(p,2)), (55,60), font, 2, greeen, 2)
            else:
                cv2.putText(frame, 'lost', (55, 60), font, 2, greeen, 2)
            rect = findcir(frame)  # 调用识别目标函数

            # print("rect=", rect)
            if rect is not None:
                flag_gate = 1
                Dxy[0] = math.atan(2.0 * (rect[0] - b1 / 2) * math.tan(35.0 * math.pi / 180.0) / b1) * 180.0 / math.pi
                dxy_num = dxy_num + 1

            else:
                flag_gate = 0
                Dxy = [0, 0]
                dxy_num = dxy_num + 1
                num = num + 1
            # print("data is 0:{}",num)
            # print(out)
            # ser.write(out)
            # out = bytearray(out)
            # print(out)
            p = Dxy[0]
            client_sk.send(bytes('#:' + agent_name + str(p) + ':s:', encoding='utf-8'))
            # time.sleep(1)
            print(agent_name, p)
            out.write(frame)
            file_txt.write(agent_name + str(p) + '\n')
            success, frame = cap.read()

            c = cv2.waitKey(2)
            if c & 0xFF == ord('q'):
                break
    cap.release()
    out.release()
