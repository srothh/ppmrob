#!/usr/bin/env python3


import cv2
import numpy as np
import matplotlib.pyplot as plt

lower_beige = np.array([0, 10, 180])
upper_beige = np.array([40, 150, 255])

# Detect lines in the image
def detect_lines(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_beige, upper_beige)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_contour_area = 4000  # Adjust as needed
    min_aspect_ratio = 0.5  # Adjust as needed
    filtered_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / h
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area
        extent = float(area) / (w * h)
        if area > min_contour_area and aspect_ratio > min_aspect_ratio and solidity > 0.6 and extent > 0.1:
            filtered_contours.append(cnt)
    cv2.drawContours(image, filtered_contours, -1, (0, 0, 255), 20)
    image = cv2.resize(image,(300,300))
    cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
    cv2.imshow('contours', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()