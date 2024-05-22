from PIL import Image
from .detect_line import detect_lines
from .detect_victim import yolo_detection


def img_processing(frame, window=None, confidence_threshhold=0.8):
    # Detect lines in the image
    # Check if a victim was detected
    # victim_detected = bool(classify_image(frame).item()) # not using classification anymore
    # Perform yolo object detection
    # Convert NumPy array to PIL Image
    pil_image = Image.fromarray(frame)
    detected = yolo_detection(pil_image, confidence_threshold=confidence_threshhold)
    lines = detect_lines(frame)
    # Display the image (needs to have image server when run from docker)
    # display_object_detection(detected, window, line_frame, 10, (0, 255, 0))
    return detected, lines
