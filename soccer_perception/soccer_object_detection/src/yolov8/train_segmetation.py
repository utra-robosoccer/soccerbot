from ultralytics import YOLO

model = YOLO("yolov8n-seg.yaml")  # build a new model from YAML

results = model.train(data="config_seg.yaml", epochs=10, imgsz=640)
