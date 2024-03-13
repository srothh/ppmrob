import cv2
import numpy as np

# Initialize VideoCapture object
cap = cv2.VideoCapture(0)

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale
    frame = cv2.resize(frame, (320, 240))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    (thresh, im_bw) = cv2.threshold(gray, 175, 255, cv2.THRESH_BINARY)

    cv2.imshow('gray', im_bw)
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(im_bw, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 250, 150)

    # Detect lines using Hough Line Transform
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=45, minLineLength=80, maxLineGap=100)
    # Draw detected lines on the original frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    # Display the frame with detected lines
    cv2.imshow('Frame with Detected Lines', frame)
    cv2.imshow('Canny', edges)

    # Check for 'q' key to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release VideoCapture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
