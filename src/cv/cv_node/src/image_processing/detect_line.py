#!/usr/bin/env python3


import cv2
import numpy as np
import matplotlib.pyplot as plt

lower_beige = np.array([0, 10, 180])
upper_beige = np.array([40, 150, 255])


# Detect lines in the image
def detect_lines(image):
    # Convert the frame to grayscale
    # For testing with different cam uncomment the 2 lines below
    # TEST ALSO GRAYSCALE
    (thresh, im_bw) = cv2.threshold(image, 175, 255, cv2.THRESH_BINARY)
    # FOR TESTING
    # cv2.imshow('gray', im_bw)
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(im_bw, (3, 3), 0)
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 250, 150)
    # Detect lines using Hough Line Transform
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=40, minLineLength=50, maxLineGap=100)
    # Draw detected lines on the original frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 3)

    # Display the frame with detected lines
    # cv2.imshow('Frame with Detected Lines', image)
    # cv2.imshow('Canny', edges)
    return lines