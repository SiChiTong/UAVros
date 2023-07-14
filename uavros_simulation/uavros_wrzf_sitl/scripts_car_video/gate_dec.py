import cv2
import os
import time
import numpy as np
from rect import minRect


def process(img):
    convert_start = time.time()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    convert_end = time.time()
    print("Convert time:{:.2f}ms".format((convert_end - convert_start) * 1000))

    thres_start = time.time()
    channels = cv2.split(hsv)
    h = channels[0]
    s = channels[1]
    v = channels[2]
    '''
    _, h1 = cv2.threshold(h, 8, 255, cv2.THRESH_BINARY_INV)
    _, h21 = cv2.threshold(h, 150, 255, cv2.THRESH_BINARY)
    _, h22 = cv2.threshold(h, 200, 255, cv2.THRESH_BINARY_INV)
    _, s = cv2.threshold(s, 25, 255, cv2.THRESH_BINARY)
    _, v = cv2.threshold(v, 40, 255, cv2.THRESH_BINARY)
    
    h2 = cv2.bitwise_and(h21, h22)
    h = cv2.bitwise_or(h1, h2)
    hs = cv2.bitwise_and(h, s)
    hsv_bin = cv2.bitwise_and(hs, v)
    '''
    _, h = cv2.threshold(h, 180, 255, cv2.THRESH_BINARY_INV)
    _, s = cv2.threshold(s, 30, 255, cv2.THRESH_BINARY_INV)
    _, v1 = cv2.threshold(v, 46, 255, cv2.THRESH_BINARY)
    _, v2 = cv2.threshold(v, 220, 255, cv2.THRESH_BINARY_INV)
    hs = cv2.bitwise_and(h, s)
    v = cv2.bitwise_and(v1, v2)
    hsv_bin = cv2.bitwise_and(hs, v)
    kernel = np.ones((1, 1), np.uint8)
    hsv_bin = cv2.erode(hsv_bin, kernel, iterations=1)
    hsv_bin = cv2.dilate(hsv_bin, kernel, iterations=1)

    thres_end = time.time()
    print("Threshold time:{:.2f}ms".format((thres_end - thres_start) * 1000))
    # cv2.imshow("hsv",hsv_bin)
    # cv2.imwrite("/result/hsv_bin.jpg",hsv_bin)

    # image, contours_ori,s hierarchy = cv2.findContours(image=hsv_bin, mode=cv2.RETR_TREE,method=cv2.CHAIN_APPROX_TC89_KCOS)
    hsv_bin, contours_ori, hierarchy = cv2.findContours(image=hsv_bin, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_TC89_KCOS)
    # print("shapepython main.py:{}".format(hierarchy.shape))

    contours_img = img.copy()
    cv2.drawContours(contours_img, contours_ori, -1, (0, 0, 255), 1)
    # cv2.imshow("contours",contours_img)

    contours = sorted(contours_ori, key=cv2.contourArea, reverse=True)[:5]

    area_list = [cv2.contourArea(x) for x in contours_ori]

    contour_img = img.copy()
    contours_res = []
    rects = []
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True)
        cnt_poly = cv2.approxPolyDP(cnt, 0.08 * cnt_len, True)
        if len(cnt_poly) == 4 and cv2.contourArea(cnt) > 10 * 10:
            contours_res.append(cnt)
            minrect = cv2.minAreaRect(cnt)
            idx = area_list.index(cv2.contourArea(cnt))
            rect = minRect(idx, minrect)
            rects.append(rect)
    result = None
    if len(rects) > 1:
        for rect in rects:
            if hierarchy[0, rect.idx, 3] == rects[0].idx:
                result = rect
                break
    elif len(rects) == 1:
        result = rects[0]
    if result is not None:
        box = np.int0(result.points)
        cv2.drawContours(contour_img, [box], 0, (0, 255, 0), 1)
    # cv2.drawContours(contour_img,contours_res,-1,(0,255,0),1)

    # for idx in range(1,len(rects)):

    cv2.imshow("contour", contour_img)
    # cv2.imwrite("/result/contour.jpg",contour_img)
    return result
