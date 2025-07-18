from typing import AnyStr, List

from soccer_msgs.msg import BoundingBox


def calc_iou(box_a, box_b):
    # determine the (x, y)-coordinates of the intersection rectangle
    x_a = max(box_a[0], box_b[0])
    y_a = max(box_a[1], box_b[1])
    x_b = min(box_a[2], box_b[2])
    y_b = min(box_a[3], box_b[3])
    # compute the area of intersection rectangle
    inter_area = max(0, x_b - x_a + 1) * max(0, y_b - y_a + 1)
    # compute the area of both the prediction and ground-truth
    # rectangles
    box_a_area = (box_a[2] - box_a[0] + 1) * (box_a[3] - box_a[1] + 1)
    box_b_area = (box_b[2] - box_b[0] + 1) * (box_b[3] - box_b[1] + 1)
    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    # return the intersection over union value
    return inter_area / float(box_a_area + box_b_area - inter_area)


def check_bounding_box(bounding_box: BoundingBox, lines: List[AnyStr], cam_width: float, cam_height: float):
    bounding_boxes = [
        bounding_box.xmin,
        bounding_box.ymin,
        bounding_box.xmax,
        bounding_box.ymax,
    ]

    best_iou = 0
    best_dimensions = None
    for line in lines:
        info = line.split(" ")
        label = int(info[0])
        if label != int(bounding_box.Class):
            continue

        x = float(info[1])
        y = float(info[2])
        width = float(info[3])
        height = float(info[4])

        xmin = int((x - width / 2) * cam_width)
        ymin = int((y - height / 2) * cam_height)
        xmax = int((x + width / 2) * cam_width)
        ymax = int((y + height / 2) * cam_height)
        ground_truth_boxes = [xmin, ymin, xmax, ymax]
        iou = calc_iou(bounding_boxes, ground_truth_boxes)
        if iou > best_iou:
            best_iou = iou
            best_dimensions = ground_truth_boxes

    return best_iou
