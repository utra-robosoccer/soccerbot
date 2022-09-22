model July14.pt:
dataset: https://universe.roboflow.com/nihal-saxena/my-ball/dataset/1
train command (need to clone yolov5 repo):
python3 train.py --img 640 --batch 8 --epochs 4 --data balls1.yaml --weights ../yolov5s.pt
