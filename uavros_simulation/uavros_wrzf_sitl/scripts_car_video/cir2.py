import cv2
import numpy as np


def findcir(img):
    hue_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low_range = np.array([0, 123, 100])
    high_range = np.array([10, 255, 255])

    # th=cv2.inRange(hue_image,low_range,high_range)

    lowMat = cv2.inRange(hue_image, np.array([0, 100, 100]), np.array([10, 255, 255]))
    upperMat = cv2.inRange(hue_image, np.array([160, 100, 100]), np.array([179, 255, 255]))

    # cv2.imshow('lowMat',lowMat)
    # cv2.imshow('upperMat',upperMat)
    redMat = cv2.addWeighted(lowMat, 1, upperMat, 1, 0)
    # cv2.imshow('redMat',redMat)
    redMat = cv2.GaussianBlur(redMat, (9, 9), 0)

    # redMat = cv2.inRange(hue_image, np.array([20, 200, 200]), np.array([30, 255, 255]))
    # redMat = cv2.GaussianBlur(redMat, (15, 15), 0)
    # cv2.imshow('Mat', redMat)
    # cv2.waitKey(10)
    # dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    # circles = cv2.HoughCircles(dilated, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=10, maxRadius=100)
    circles = cv2.HoughCircles(redMat, cv2.HOUGH_GRADIENT, 1, 100, param1=50, param2=10, minRadius=10, maxRadius=100)
    result = [0, 0]
    flag = 0
    if circles is not None:
        x, y, radius = circles[0][0]
        center = (x, y)
        cv2.circle(img, center, int(radius), (0, 255, 0), 2)
        result[0] = x
        result[1] = y
        flag = 1
    cv2.imshow('RED', img)
    cv2.waitKey(10)
    # cv2.imshow('result', img)
    return flag, result
