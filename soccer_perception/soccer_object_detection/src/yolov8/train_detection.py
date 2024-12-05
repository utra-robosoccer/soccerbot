import torch
from ultralytics import YOLO

# Check for CUDA device and set it
# print(torch.__version__)
torch.cuda.set_device(0)
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Now run inference or training
# Load a model
# model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8n.pt").to(device)  # load a pretrained model (recommended for training)

# Use the model
model.train(data="config.yaml", epochs=8)  # train the model
metrics = model.val()  # evaluate model performance on the validation set
# results = model("https://ultralytics.com/images/bus.jpg")  # predict on an image
# path = model.export(format="onnx")  # export the model to ONNX format
