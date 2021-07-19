#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: decision.py
Date: 2021/5/26 11:08
LastEditors: LI Jinjie
LastEditTime: 2021/5/26 11:08
Description: receive the states of cars and uavs, and return the action.
'''
import numpy as np
import quadrotor.cmd as cmd
import quadrotor.parameters as para


class DecisionMaker:
    """
    检测每架飞机所处的状态，进行对五架无人机全部进行的操作。
    """

    def __init__(self, method, cars=[2, 2, 1], detection_save_num=5):
        """

        :param method: 'stable,4', 'max,5', 'stable,5'
        """
        # 准备状态检测
        self.ready_dict = None
        self.reset_ready_dict()
        self.destinations = {}

        # 决策表
        self.uav_order = [para.UAV_AC_NAME, para.UAV_A1_NAME, para.UAV_A2_NAME, para.UAV_SM_NAME, para.UAV_SS_NAME]
        tab = {}
        if method == 'max,5':  # 车辆数字和为5，得分最多的方案
            # tab['car1,car2,car3'] = [uav_ac,uav_a1,uav_s1,uav_s2,uav_s3]
            # 1，2，3分别表明uav选择靠近的car
            tab['023'] = [3, 2, 2, 3, 3]
            tab['032'] = [3, 2, 2, 3, 2]
            tab['113'] = [3, 2, 1, 3, 3]
            tab['122'] = [3, 2, 2, 3, 1]
            tab['131'] = [2, 1, 2, 3, 2]
            tab['203'] = [3, 1, 1, 3, 3]
            tab['212'] = [3, 1, 1, 3, 2]
            tab['221'] = [2, 1, 1, 2, 3]
            tab['230'] = [2, 1, 1, 2, 2]
            tab['302'] = [3, 1, 1, 3, 1]
            tab['311'] = [3, 1, 1, 2, 1]
            tab['320'] = [2, 1, 1, 2, 1]
        elif method == 'stable,5':  # 车辆数字和为5，得分稳定的方案
            # tab['car1,car2,car3'] = [uav_ac,uav_a1,uav_a2,uav_sm,uav_ss]
            # 1分别表明uav_ac,uav_a1,uav_a2下降，uav_sm靠近2，uav_ss接近；0则代表上升，靠近3，远离
            tab['023'] = [1, 0, 1, 1, 0]
            tab['032'] = [1, 0, 1, 0, 0]
            tab['113'] = [1, 1, 1, 1, 1]
            tab['122'] = [1, 1, 1, 1, 1]
            tab['131'] = [1, 1, 1, 0, 1]
            tab['203'] = [0, 1, 1, 0, 1]
            tab['212'] = [1, 1, 1, 0, 1]
            tab['221'] = [1, 1, 1, 1, 1]
            tab['230'] = [1, 1, 0, 1, 1]
            tab['302'] = [0, 1, 1, 0, 1]
            tab['311'] = [1, 1, 1, 1, 1]
            tab['320'] = [1, 1, 0, 1, 1]
        elif method == 'lqd,5':  # 车辆数字和为5，李老师给出的方案
            # tab['car1,car2,car3'] = [uav_a1,uav_ac,uav_a2,uav_s1,uav_s2]
            # 对uav_a来说，1表示下降，0表示上升
            # 对uav_s来说，1，2，3分别表明uav选择靠近的car
            tab['023'] = [0, 1, 1, 3, 3]
            tab['032'] = [0, 1, 1, 2, 2]
            tab['113'] = [1, 1, 1, 3, 3]
            tab['122'] = [1, 1, 1, 2, 3]
            tab['131'] = [1, 1, 1, 2, 2]
            tab['203'] = [1, 0, 1, 3, 3]
            tab['212'] = [1, 1, 1, 1, 3]
            tab['221'] = [1, 1, 1, 1, 2]
            tab['230'] = [1, 1, 0, 2, 2]
            tab['302'] = [1, 0, 1, 1, 1]
            tab['311'] = [1, 1, 1, 1, 1]
            tab['320'] = [1, 1, 0, 1, 1]
        elif method == 'fixed,score=3.25':  # 车辆数字和为5，侧面在动，左边2架都能动，上下固定
            # tab['car1,car2,car3'] = [uav_a1,uav_ac,uav_a2,uav_sm,uav_ss]
            # 对uav_a1,uav_ac来说，-1表示固定下降
            # 对uav_a2,uav_sm,uav_ss来说，1，2，3分别表明uav选择靠近的car
            tab['023'] = [-1, -1, 3, 2, 2]
            tab['032'] = [-1, -1, 3, 2, 2]
            tab['113'] = [-1, -1, 2, 1, 1]
            tab['122'] = [-1, -1, 3, 2, 3]
            tab['131'] = [-1, -1, 3, 2, 2]
            tab['203'] = [-1, -1, 3, 1, 1]
            tab['212'] = [-1, -1, 3, 1, 3]
            tab['221'] = [-1, -1, 3, 2, 1]
            tab['230'] = [-1, -1, 2, 1, 2]
            tab['302'] = [-1, -1, 3, 1, 1]
            tab['311'] = [-1, -1, 3, 1, 1]
            tab['320'] = [-1, -1, 2, 1, 1]
        elif method == 'fixed,score=3':  # 车辆数字和为5，只有侧面在动
            # tab['car1,car2,car3'] = [uav_a1,uav_ac,uav_a2,uav_sm,uav_ss]
            # 对uav_a1,uav_ac,uav_a2,来说，-1表示固定下降
            # 对uav_sm,uav_ss来说，1，2，3分别表明uav选择靠近的car
            tab['023'] = [-1, -1, -1, 3, 3]
            tab['032'] = [-1, -1, -1, 2, 2]
            tab['113'] = [-1, -1, -1, 3, 3]
            tab['122'] = [-1, -1, -1, 2, 3]
            tab['131'] = [-1, -1, -1, 2, 2]
            tab['203'] = [-1, -1, -1, 3, 3]
            tab['212'] = [-1, -1, -1, 1, 3]
            tab['221'] = [-1, -1, -1, 1, 2]
            tab['230'] = [-1, -1, -1, 2, 2]
            tab['302'] = [-1, -1, -1, 1, 1]
            tab['311'] = [-1, -1, -1, 1, 1]
            tab['320'] = [-1, -1, -1, 1, 1]
        self.decision_tab = tab

        # 无人机跟踪向量，字典形式      2,2,1
        self.uav_track_which_car = {para.UAV_A1_NAME: 0,
                                    para.UAV_AC_NAME: 1,
                                    para.UAV_A2_NAME: 2,
                                    para.UAV_SS_NAME: 1,
                                    para.UAV_SM_NAME: 1}

        # 检测相关
        self.car_num_detections = {0: (np.ones(detection_save_num) * cars[0]).tolist(),
                                   1: (np.ones(detection_save_num) * cars[1]).tolist(),
                                   2: (np.ones(detection_save_num) * cars[
                                       2]).tolist()}  # 1个dict，每个元素是一个list，储存最近5次检测的车数字结果
        self.car_num_real = cars  # 1个list，储存实际的车数字的数据
        self.new_detection_flag = False  # 判断是否要更新车数字的状态
        self.update_num = 0

    # 重设为False
    def reset_ready_dict(self, mode=para.MODE_0):
        if mode == para.MODE_0:
            self.ready_dict = {para.UAV_A1_NAME: True, para.UAV_A2_NAME: True, para.UAV_AC_NAME: False,
                               para.UAV_SM_NAME: True, para.UAV_SS_NAME: True, para.GROUND_STATION_NAME: False}
        elif mode == para.MODE_2:
            self.ready_dict = {para.UAV_SM_NAME: False, para.UAV_SS_NAME: False}

    def all_ready(self):
        if sum(self.ready_dict.values()) == len(self.ready_dict):
            return True
        else:
            return False

    def make_decision(self, car_num):
        """
        接收汽车状态，返回每个无人机的动作
        :param car_num: 一个list, 如[0, 1, 2]
        :return: 一个list，如[1, 0, 0, 1, 1]
        """
        # 只需要三辆车的数字
        index = 0
        for num in car_num:
            index = 10 * index
            index += num
        if index < 100:
            idx = '0' + str(index)
        else:
            idx = str(index)
        return self.decision_tab[idx]
