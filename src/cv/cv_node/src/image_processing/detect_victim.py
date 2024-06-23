#!/usr/bin/env python3
import torch
from torchvision import transforms
from PIL import Image
import cv2
from torchvision.transforms import Compose, Resize, ToTensor
from .SRCnn import SRCnn
import os
model = SRCnn(3, 32, 1)
device = "cuda" if torch.cuda.is_available() else "cpu"
model = model.to(device)
# Load the trained model
script_dir = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(script_dir, 'model', 'first_cnn.pth')
yolo_path = os.path.join(script_dir, 'model', 'best.pt')
state_dict = torch.load(model_path, map_location=torch.device(device))
model.load_state_dict(state_dict)
# Load the state dictionary into the model
model.load_state_dict(state_dict)
model.eval()
# Define the image transformation process
transformation = Compose([
    Resize((64, 64)),
    ToTensor()
])


def classify_image(frame):
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    image = transformation(image)
    image = image.unsqueeze(0)
    image = image.to(device)
    output = model(image)
    # Get the predicted class
    logits = torch.sigmoid(output)
    predicted = (logits > 0.5).long()
    return predicted


# Performs object detection with our YOLO model.
# Requires a PIL image for the model task
def yolo_detection(frame, yolo_model, confidence_threshold=0.9):
    results = yolo_model(frame)  # Perform inference
    filtered_results = results.xyxy[0][
        results.xyxy[0][:, 4] > confidence_threshold]  # Filter detections by confidence threshold
    return filtered_results
