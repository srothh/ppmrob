import cv2
def display_image(image,victim_detected, window):
    text = "Victim Detected" if victim_detected else "No Victim Detected"
    color = (0, 255, 0) if victim_detected else (0, 0, 255)
    cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow(window, image)
    cv2.waitKey(1)

# Draw results of YOLO object detection.
# Requires cv2 RGB Image.
def display_object_detection(detected, window, rgb_image, thickness, color):
    for detection in detected:
        x_min, y_min, x_max, y_max, confidence, class_idx = detection.tolist()  # Convert tensor to list

        # Draw bounding box
        start_point = (int(x_min), int(y_min))  # Top-left corner
        end_point = (int(x_max), int(y_max))  # Bottom-right corner
        color = color  # Green color
        thickness = thickness  # Line thickness
        cv2.rectangle(rgb_image, start_point, end_point, color, thickness)

        # Display class label and confidence
        label_text = f'Class: {"Victim"}, Confidence: {confidence:.2f}'
        cv2.putText(rgb_image, label_text, start_point, cv2.FONT_HERSHEY_SIMPLEX, 2.5, color, 1)

    # Display the image with bounding boxes
    cv2.imshow(window, rgb_image[:, :, ::-1])  # Convert PIL image to numpy array and display with OpenCV
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()  # Close OpenCV window
