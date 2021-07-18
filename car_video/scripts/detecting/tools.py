#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@Author  : Li-Jinjie
@Time    : 2021/7/17 12:28
@File    : finding_corners
@Email   : lijinjie362@outlook.com
@Description:  None
'''
import cv2
import numpy as np


def find_boxes_using_contours(img_mor):
    """
    hO
    :param img_mor:
    :return:
    """
    contours, hierarchy = cv2.findContours(img_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes_list = []
    centers_list = []
    side_lengths_list = []
    degrees_list = []
    for contour in contours:
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
        LOWER_BOUND = 1.0
        UPPER_BOUND = 2.6
        if (LOWER_BOUND <= 1 / ar <= UPPER_BOUND or LOWER_BOUND <= ar <= UPPER_BOUND) and area > 500:
            boxes_list.append(box)
            # save
            centers_list.append(center)
            side_lengths_list.append(side_length)
            degrees_list.append(degree)

    # # ====== to display ========
    # img_with_pts = img_sub_norm.copy()
    # for corners in corners_list:
    #     for i in range(corners.shape[0]):
    #         cv2.circle(img_with_pts, (corners[i, 0, :].item(0), corners[i, 0, :].item(1)), 2, 255, -1)
    # cv2.imshow("imgWithPoints", img_with_pts)
    # cv2.waitKey(0)
    return boxes_list, centers_list, side_lengths_list, degrees_list


def otsu(gray):
    """
    otsu
    :param gray: a gray img
    :return: threshold
    """
    pixel_number = gray.shape[0] * gray.shape[1]
    mean_weight = 1.0/pixel_number
    his, bins = np.histogram(gray, np.arange(0,257))
    final_thresh = -1
    final_value = -1
    intensity_arr = np.arange(256)
    for t in bins[1:-1]: # This goes from 1 to 254 uint8 range (Pretty sure wont be those values)
        pcb = np.sum(his[:t])
        pcf = np.sum(his[t:])
        Wb = pcb * mean_weight
        Wf = pcf * mean_weight

        mub = np.sum(intensity_arr[:t] * his[:t]) / float(pcb)
        muf = np.sum(intensity_arr[t:] * his[t:]) / float(pcf)
        #print mub, muf
        value = Wb * Wf * (mub - muf) ** 2

        if value > final_value:
            final_thresh = t
            final_value = value
    # final_img = gray.copy()
    # print(final_thresh)
    # final_img[gray > final_thresh] = 255
    # final_img[gray < final_thresh] = 0

    return final_thresh
