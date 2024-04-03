import pathlib

import cv2
import numpy as np
import torch
from PIL import Image

# Initialize VideoCapture object
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

model = torch.hub.load('ultralytics/yolov5', 'custom', path='model/best.pt', force_reload=True)  # local model
for i in range(17,85):
    if i < 10:
        str = f"test/frame-000{i}.png"
    else:
        str = f"test/frame-00{i}.png"
    frame = cv2.imread(str)
    img_victim = Image.fromarray(frame)  # Convert BGR to RGB
    # Convert the frame to grayscale
    results = model(img_victim)
    filtered_results = results.xyxy[0][
        results.xyxy[0][:, 4] > 0.75]  # Filter detections by confidence threshold

    (thresh, im_bw) = cv2.threshold(frame, 175, 255, cv2.THRESH_BINARY)
    cv2.imshow('gray', frame)
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 150, 150)
    # Detect lines using Hough Line Transform
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=40, minLineLength=50, maxLineGap=100)
    # Draw detected lines on the original frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    for result in filtered_results:  # Accessing bounding boxes and labels
        label = int(result[5])  # Convert label to integer
        confidence = result[4]
        box = result[:4].cpu().numpy().astype(int).tolist()  # Convert box coordinates to list of integers
        # Draw bounding box on the frame
        start_point = (box[0]), int(box[1])  # Top-left corner
        end_point = (box[2]), int(box[3])  # Bottom-right corner
        color = (0, 255, 0)  # Green color
        thickness = 2  # Line thickness
        cv2.rectangle(frame, start_point, end_point, color, thickness)
        # Display class label and confidence
        label_text = f'{"Victim"}: {confidence:.2f}'
        cv2.putText(frame, label_text, start_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    # Display the frame with detected lines
    cv2.imshow('Frame with Detected Lines', frame)
    cv2.imshow('Canny', edges)
    cv2.waitKey(0)
    # Check for 'q' key to quit
    # Release VideoCapture and close all OpenCV windows
