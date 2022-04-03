import json
import os

import cv2
import tqdm

input_folder = "/home/nam/dataset/ball_bounding_boxes_positive"
output_folder = "/home/nam/dataset/ball_bounding_boxes_positive_small"


def process_img(img_name):
    path = os.path.join(input_folder, img_name)

    img = cv2.imread(path)
    img = img[:, :, :3]  # get rid of alpha channel
    scale = 480 / 300  # ~3.6
    dim = (int(640 / scale), 300)  # (640, 480) -> (?, 300)
    img = cv2.resize(img, dsize=dim, interpolation=cv2.INTER_AREA)

    w, h = 400, 300
    y, x, _ = img.shape
    x_offset = x / 2 - w / 2
    y_offset = y / 2 - h / 2

    crop_img = img[int(y_offset) : int(y_offset + h), int(x_offset) : int(x_offset + w)]

    new_path = os.path.join(output_folder, img_name)
    cv2.imwrite(new_path, crop_img)


def main():
    try:
        os.mkdir(output_folder)
    except OSError:
        print("Creation of the directory %s failed" % output_folder)
    else:
        print("Successfully created the directory %s " % output_folder)

    files = os.listdir(input_folder)
    for i in tqdm.tqdm(range(len(files)), desc="Resizing Images"):
        img_name = files[i]
        if ".json" in img_name:
            with open(os.path.join(input_folder, img_name), "r") as annot:
                val = json.load(annot)
                [[x1, y1], [x2, y2]] = val

                w, h = 400, 300
                x, y = 640, 480
                scale = y / h
                x_offset = x / scale / 2 - w / 2

                x1 = int(x1 / scale - x_offset)
                y1 = int(y1 / scale)
                x2 = int(x2 / scale - x_offset)
                y2 = int(y2 / scale)

                print("label::ball|{}.jpg|0|0|{}|{}|{}|{}|0|0|0|0".format(img_name[3:-5], x1, y1, x2, y2))
        if ".jpg" in img_name and "border" not in img_name:
            # print(img_name)
            process_img(img_name)


if __name__ == "__main__":
    main()
