#!/usr/bin/env python3
import torch
from torchvision import transforms
from PIL import Image
import cv2
from torchvision.transforms import Compose, Resize, ToTensor

from SRCnn import SRCnn

model = SRCnn(3, 32, 1)
device = "cuda" if torch.cuda.is_available() else "cpu"
model = model.to(device)
# Load the trained model
state_dict = torch.load('../../model/first_cnn.pth')

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
