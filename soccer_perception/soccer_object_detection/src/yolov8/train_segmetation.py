import torch
from ultralytics import YOLO

model = YOLO("yolov8n-seg.pt")  # build a new model from YAML

# torch.cuda.set_device(0)
# device = 'cuda' if torch.cuda.is_available() else 'cpu'
# print(f'Using device: {device}')
print(torch.__version__)

torch.cuda.empty_cache()
print(torch.cuda.is_available())

results = model.train(data="config_seg.yaml", epochs=100, imgsz=640, batch=-1, device=0, amp=True, cache=False)  # -1 for auto batch size
# reality train, simulation test, reality test (caused problems)
