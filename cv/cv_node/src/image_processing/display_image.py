import cv2


def display_image(image,victim_detected):
    text = "Victim Detected" if victim_detected else "No Victim Detected"
    cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
    cv2.imshow('contours', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()