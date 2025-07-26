import os
import sys
from os.path import expanduser

import cv2
import matplotlib
import numpy as np

matplotlib.use("agg")
import matplotlib.pyplot as plt
import torch
from segment_anything import SamPredictor, sam_model_registry


def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)


def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels == 1]
    neg_points = coords[labels == 0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color="green", marker="*", s=marker_size, edgecolor="white", linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color="red", marker="*", s=marker_size, edgecolor="white", linewidth=1.25)


def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor="green", facecolor=(0, 0, 0, 0), lw=2))


src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
test_path = src_path + "data/images/simulation"
model_path = src_path + "soccer_object_detection/models/half_5.pt"

model2 = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)
matplotlib.use("TkAgg")  # Change backend after loading model

# set model params
# initialize the model
sam_checkpoint = "sam_vit_b_01ec64.pth"
model_type = "vit_b"
sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
predictor = SamPredictor(sam)

for file_name in os.listdir(f"{test_path}/images"):
    img = cv2.imread(os.path.join(f"{test_path}/images", file_name))
    img = cv2.resize(img, dsize=(640, 480))
    results2 = model2(img)
    l = []
    for prediction in results2.xyxy[0]:
        x1, y1, x2, y2, confidence, img_class = prediction.cpu().numpy()
        l.append([round(x1), round(y1), round(x2), round(y2)])
    print(l)
    if len(l) == 0:
        continue
    input_boxes = torch.tensor(l, device=predictor.device)

    predictor.set_image(img)
    # input_box = np.array(l[0])
    # input_box = np.array([125, 100, 300, 375])
    #
    # masks, _, _ = predictor.predict(box=input_box[None, :])
    # plt.figure(figsize=(10, 10))
    # plt.imshow(img)
    # show_mask(masks[0], plt.gca())
    # show_box(input_box, plt.gca())
    # plt.axis('off')
    # plt.show()
    transformed_boxes = predictor.transform.apply_boxes_torch(input_boxes, img.shape[:2])
    masks, _, _ = predictor.predict_torch(point_coords=None, point_labels=None, boxes=transformed_boxes, multimask_output=False)

    plt.figure(figsize=(10, 10))
    plt.imshow(img)
    for mask in masks:
        show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
    for box in input_boxes:
        show_box(box.cpu().numpy(), plt.gca())
    plt.axis("off")
    plt.show()
    cv2.waitKey()
    # cv2.destroyAllWindows()
    # break
