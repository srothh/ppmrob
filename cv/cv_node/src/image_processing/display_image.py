import cv2
def display_image(image,victim_detected, window):
    text = "Victim Detected" if victim_detected else "No Victim Detected"
    color = (0, 255, 0) if victim_detected else (0, 0, 255)
    cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow(window, image)
    cv2.waitKey(1)
