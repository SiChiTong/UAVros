#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: detect_car.py.py
Date: 2021/7/17 11:01
LastEditors: LI Jinjie
LastEditTime: 2021/7/17 11:01
Description: file content
'''

# coding=utf-8
import cv2
import numpy as np
from detecting.tools import find_boxes_using_contours

# TODO: 解决识别存在的两个问题
#  1. 出现的多目标：选择与跟踪结果重叠最多的。  方法：保持上一次的检测结果
#  2. 暂时的跳变


class CarDetector:
    def __init__(self):
        self.box_pre = None
        self.area_pre = None

    def detect_car(self, img):
        """
        检测车辆
        :param img: 彩色BGR图片
        :return:
        boxes_list: 方框的两个角点坐标
        centers_list: 中心点坐标
        side_lengths_list: 边长
        degrees_list: 转角
        """
        (img_w, img_h, _) = img.shape

        # 1. 预处理
        img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Otsu's thresholding after Gaussian filtering
        blur = cv2.GaussianBlur(img_grey, (5, 5), 0)

        # 归一化    不能归一化，要考虑没有白车的情况
        # img_blur_int = blur.astype(np.int32)
        # img_blur_norm = ((img_blur_int - np.min(img_blur_int)) * 255 / (
        #         np.max(img_blur_int) - np.min(img_blur_int))).astype(np.uint8)

        # 2. 二值化
        ret, img_bw = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)  # TODO: 调节阈值

        # thresh = otsu(blur)  # 大津阈值效果不好，不稳定。
        # thresh_final = thresh + (255 - thresh) * 0.70
        # ret, img_bw = cv2.threshold(blur, thresh_final, 255, cv2.THRESH_BINARY)

        # 3. 形态学 降噪，联通
        # 形态学 先开运算降噪，再闭运算联通
        kernel = np.ones((6, 6), np.uint8)
        img_mor = cv2.morphologyEx(img_bw, cv2.MORPH_OPEN, kernel)

        kernel = np.ones((15, 15), np.uint8)  # need to adjust more carefully
        img_mor = cv2.morphologyEx(img_mor, cv2.MORPH_CLOSE, kernel)

        # 4. 轮廓检测
        # 选择轮廓，根据矩形进行选择
        img_mor, contours, hierarchy = cv2.findContours(img_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes_list = []
        centers_list = []
        side_lengths_list = []
        degrees_list = []
        for contour in contours:  # 存在检测
            rect = cv2.minAreaRect(contour)
            # Each rectangle is specified by the center point (mass center),
            # length of each side (represented by Size2f structure) and the rotation angle in degrees.

            center = rect[0]
            side_length = rect[1]
            (w, h) = side_length
            degree = rect[2]

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            ar = w / float(h)
            area = w * h

            # TODO: 修改判断条件
            LOWER_BOUND = 1.0  # 比例
            UPPER_BOUND = 2.6

            AREA_TH = 0.01  # 面积

            area_all_pixel = img_w * img_h
            if (LOWER_BOUND <= 1 / ar <= UPPER_BOUND or LOWER_BOUND <= ar <= UPPER_BOUND) \
                    and area / area_all_pixel > AREA_TH:
                # save
                boxes_list.append(box)
                centers_list.append(center)
                side_lengths_list.append(side_length)
                degrees_list.append(degree)

        # 5. 筛选结果
        if len(centers_list) != 0:  # 有检测结果的情况才执行
            c_w = img_w / 2.
            c_h = img_h / 2.
            # 判断两个矩形有没有交叉
            if self.box_pre is not None:
                overleap_rect = {}  # idx: overleap area
                black = np.zeros(img_grey.shape, dtype=np.uint8)  # 背景
                cv2.drawContours(black, [self.box_pre], 0, 255, thickness=-1)  # 填充，画上一帧的方框
                for idx, box in enumerate(boxes_list):
                    img_tmp = black.copy()
                    cv2.drawContours(img_tmp, [box], 0, 255, thickness=-1)
                    _, contours, _ = cv2.findContours(img_tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours) == 1:  # 如果交叉
                        area_now = side_lengths_list[idx][0] * side_lengths_list[idx][1]
                        area_total = cv2.contourArea(contours[0])
                        overleap_rect[idx] = self.area_pre + area_now - area_total  # 保存数据

                # 如果存在交叉的矩形，找到交叉最多的矩形对应的idx
                if len(overleap_rect) > 0:
                    min_idx = min(overleap_rect, key=overleap_rect.get)
                    self.box_pre = boxes_list[min_idx]
                    self.area_pre = side_lengths_list[min_idx][0] * side_lengths_list[min_idx][1]
                    return boxes_list[min_idx], centers_list[min_idx], side_lengths_list[min_idx], degrees_list[min_idx]

            # 如果不满足上述情况，根据检测结果距中心的距离选择一个框
            min_distance = 9999
            min_idx = 0
            for idx, center in enumerate(centers_list):
                distance = np.sqrt((center[0] - c_w) ** 2 + (center[1] - c_h) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    min_idx = idx
            self.box_pre = boxes_list[min_idx]
            self.area_pre = side_lengths_list[min_idx][0] * side_lengths_list[min_idx][1]
            return boxes_list[min_idx], centers_list[min_idx], side_lengths_list[min_idx], degrees_list[min_idx]
        else:  # 没检测到，全部返回None
            return None, None, None, None



    # for debugging
    # img_quads_detect = img.copy()
    # for box in boxes_list:
    #     cv2.drawContours(img_quads_detect, [box], 0, (0, 0, 255), 2)

    # cv2.imshow("img", img)
    # cv2.imshow("img_blur_norm", img_blur_norm)
    # cv2.imshow("img_bw", img_bw)
    # cv2.imshow("img_mor", img_mor)
    # cv2.imshow("quads detection", img_quads_detect)
    # cv2.waitKey(10)
