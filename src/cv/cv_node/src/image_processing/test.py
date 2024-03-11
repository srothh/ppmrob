import cv2
import torch
from PIL import Image
import pathlib
import numpy as np
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

# Load custom YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='model/best.pt', force_reload=True)  # local model
cap = cv2.VideoCapture(0)  # Open the default camera

while True:
    ret, frame = cap.read()  # Read a frame from the video stream

    if ret:  # Check if frame was successfully read
        # Preprocess the frame
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        img = Image.fromarray(img)  # Convert to PIL Image

        # Run inference with YOLOv5
        results = model(img)  # Perform inference

        # Process the detection results
        filtered_results = results.xyxy[0][
            results.xyxy[0][:, 4] > 0.85]  # Filter detections by confidence threshold
        for result in filtered_results:  # Accessing bounding boxes and labels
            label = int(result[5])  # Convert label to integer
            confidence = result[4]
            box = result[:4].cpu().numpy().astype(int).tolist()  # Convert box coordinates to list of integers

            # Draw bounding box on the frame
            start_point = (box[0]), int(box[1])  # Top-left corner
            end_point = (box[2]), int(box[3])  # Bottom-right corner
            print(start_point)
            print(end_point)
            color = (0, 255, 0)  # Green color
            thickness = 2  # Line thickness
            cv2.rectangle(frame, start_point, end_point, color, thickness)

            # Display class label and confidence
            label_text = f'{model.names[label]}: {confidence:.2f}'
            cv2.putText(frame, label_text, start_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Display the frame with bounding boxes
        cv2.imshow('YOLOv5 Object Detection', frame)

    # Exit loop if 'q' is pressed or if ret is False (stream ended)
    if cv2.waitKey(1) & 0xFF == ord('q') or not ret:
        break

cap.release()  # Release the video capture object
cv2.destroyAllWindows()  # Close all OpenCV windows
