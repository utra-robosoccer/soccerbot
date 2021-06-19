import os
import copy
import cv2
import numpy as np
import torch
import torchvision
import yaml
from PIL import Image
from torch.utils.data import Dataset, DataLoader, random_split
from model import Label, find_batch_bounding_boxes
import util

train_path = '/media/nam/My Passport/bit-bots-ball-dataset-2018/train'
test_path = '/media/nam/My Passport/bit-bots-ball-dataset-2018/test'


def initialize_loader(batch_size, jitter=[0, 0, 0, 0], num_workers=64, shuffle=True):
    train_folders = [os.path.join(train_path, folder) for folder in os.listdir(train_path)]
    test_folders = [os.path.join(test_path, folder) for folder in os.listdir(test_path)]

    full_dataset = MyDataSet(train_folders, (150, 200), jitter=jitter)
    test_dataset = MyDataSet(test_folders, (150, 200))

    train_size = int(0.8 * len(full_dataset))
    valid_size = len(full_dataset) - train_size
    train_dataset, valid_dataset = random_split(full_dataset, [train_size, valid_size])

    train_loader = DataLoader(train_dataset,
                              batch_size=batch_size,
                              num_workers=num_workers,
                              shuffle=shuffle)
    valid_loader = DataLoader(valid_dataset,
                              batch_size=batch_size,
                              num_workers=num_workers,
                              shuffle=shuffle)
    test_loader = DataLoader(test_dataset,
                             batch_size=batch_size,
                             num_workers=num_workers,
                             shuffle=False)

    full_dataset.num_train_ball_labels, full_dataset.num_train_robot_labels = util.subset_label_count(train_dataset,
                                                                                                      Label.BALL,
                                                                                                      Label.ROBOT)
    valid_ball, valid_robot = util.subset_label_count(valid_dataset,
                                                      Label.BALL,
                                                      Label.ROBOT)

    assert full_dataset.num_ball_labels == full_dataset.num_train_ball_labels + valid_ball
    assert full_dataset.num_robot_labels == full_dataset.num_train_robot_labels + valid_robot

    print('full dataset: # images {:>6}, # robots {:>6}, # balls {:>6}'.format(
        len(full_dataset),
        full_dataset.num_robot_labels,
        full_dataset.num_ball_labels
    ))

    print('   train dataset: # images {:>6}, # robots {:>6}, # balls {:>6}'.format(
        len(train_dataset),
        full_dataset.num_train_robot_labels,
        full_dataset.num_train_ball_labels,
    ))

    print('   valid dataset: # images {:>6}, # robots {:>6}, # balls {:>6}'.format(
        len(valid_dataset),
        valid_robot,
        valid_ball,
    ))

    print('test dataset:  # images {:>6}, # robots {:>6}, # balls {:>6}'.format(
        len(test_dataset),
        test_dataset.num_robot_labels,
        test_dataset.num_ball_labels
    ))

    return (train_loader, valid_loader, test_loader), (full_dataset, test_dataset)


class MyDataSet(Dataset):
    def __init__(self, folder_paths, target_dim, jitter=[0, 0, 0, 0]):
        self.folder_paths = folder_paths  # folders of the images
        self.img_paths = []  # all individual image paths
        self.bounding_boxes = {}  # image paths and their labels
        self.target_height = target_dim[0]
        self.target_width = target_dim[1]
        # seems useful: https://datduyng.github.io/2019/03/20/data-augmentation-for-semantic-segmantation-with-pytorch.html
        self.img_transform = torchvision.transforms.Compose([
            torchvision.transforms.ColorJitter(brightness=jitter[0], contrast=jitter[1], saturation=jitter[2],
                                               hue=jitter[3]),
            torchvision.transforms.Resize(self.target_height, interpolation=Image.BILINEAR),
            torchvision.transforms.CenterCrop((self.target_height, self.target_width)),
        ])
        self.mask_transform = torchvision.transforms.Compose([
            torchvision.transforms.Resize(self.target_height, interpolation=Image.NEAREST),
            torchvision.transforms.CenterCrop((self.target_height, self.target_width)),
        ])

        # label statistics
        self.num_ball_labels = 0  # for full dataset
        self.num_robot_labels = 0
        self.num_train_robot_labels = 0  # for subset of full dataset
        self.num_train_ball_labels = 0

        # add paths for train data with labels
        for path in folder_paths:
            for file in os.listdir(path):
                if '.txt' in file:
                    file_labels = os.path.join(path, file)
                    self.read_labels(path, file_labels, 'txt')
                elif '.yaml' in file:
                    file_labels = os.path.join(path, file)
                    self.read_labels(path, file, 'yaml')

    def read_labels(self, path, file_labels, file_type):
        """
        :param path: folder containing images and label text file
        :param file_labels: label text file
        :param file_type: processes the file depending on whether it is the original .txt or the BitBots .yaml file
        :return: None
        """

        with open(file_labels) as labels:
            if file_type == 'txt':
                for i, line in enumerate(labels):
                    if i <= 5:  # ignore first few metadata lines
                        continue

                    try:
                        label, img, _, _, x1, y1, x2, y2, _, _, _, _ = line.split('|')
                    except:
                        # ignore unknown format
                        continue

                    if label == 'label::ball':
                        label = Label.BALL
                        self.num_ball_labels += 1
                    elif label == 'label::robot':
                        label = Label.ROBOT
                        self.num_robot_labels += 1
                    else:
                        print('Unexpected Label:', label)

                    img_path = os.path.join(path, img)
                    if img_path not in self.img_paths:
                        self.bounding_boxes[img_path] = []
                        self.img_paths.append(img_path)

                    self.bounding_boxes[img_path].append([int(x1), int(y1), int(x2), int(y2), label])

            elif file_type == 'yaml':
                documents = yaml.full_load(labels)
                image_list = documents['images']
                for image in image_list:
                    annotations = image_list[image]['annotations']
                    for annotation in annotations:
                        if annotation['in_image'] == True:
                            if annotation['type'] == 'ball' or annotation['type'] == 'robot':
                                location = annotation['vector'] #[[x1,y1],[x2,y2]]
                                x1 = location[0][0]
                                y1 = location[0][1]
                                x2 = location[1][0]
                                y2 = location[1][1]

                                if annotation['type'] == 'ball':
                                    label = label.BALL
                                    self.num_ball_labels += 1
                                else:
                                    label = label.ROBOT
                                    self.num_robot_labels += 1

                                img_path = os.path.join(path, image)
                                if img_path not in self.img_paths:
                                    self.bounding_boxes[img_path] = []
                                    self.img_paths.append(img_path)

                                self.bounding_boxes[img_path].append([int(x1), int(y1), int(x2), int(y2), label])


    def __len__(self):
        return len(self.bounding_boxes)

    def __getitem__(self, index):
        """
        :param index: index of data point
        :return: img ndarray (3 x w x h) RGB image
                 mask ndarray (w x h) segmentation classification of each pixel
                 index (int) image index
        """
        img_path = self.img_paths[index]
        bounding_boxes = self.bounding_boxes[img_path]
        img = util.read_image(img_path)

        height, width, _ = np.array(img).shape
        # final mask will have no channels but we need 3 initially to convert it to PIL image to apply transformation
        mask = np.ones((height, width, 3)) * Label.OTHER.value
        for bb in bounding_boxes:
            pt1, pt2, label = np.array(bb[0:2]), np.array(bb[2:4]), bb[4]

            center = tuple(((pt1 + pt2) / 2).astype(np.int))
            size = tuple(((pt2 - pt1) / 2).astype(np.int))

            if not size == (0, 0):
                if label == Label.BALL:
                    mask = cv2.ellipse(mask, center, size, 0, 0, 360, label.value, -1)
                if label == Label.ROBOT:
                    mask = cv2.rectangle(mask, tuple(pt1), tuple(pt2), label.value, -1)

        mask = Image.fromarray(mask.astype('uint8'))

        # Apply transformations to get desired dimensions
        img = np.array(self.img_transform(img))
        mask = np.array(self.mask_transform(mask))

        # flip to channel*W*H - how Pytorch expects it
        img = np.moveaxis(img, -1, 0)
        mask = np.moveaxis(mask, -1, 0)[0]  # get rid of channel dimension

        return img, mask, index

    def visualize_images(self, start=0, end=None, delay=10, scale=4, model=None):
        """ display dataset as video sequence
        :param start: start frame
        :param end: end frame
        :param delay: time dealy between displaying frames
        :param scale: resize frames
        :param model: model
        :return: None
        """
        if end is None:
            end = len(self)
        self.img_paths = list(sorted(self.img_paths))  # we want names to be sorted so that they are displayed in order
        for ind in range(start, end):
            img, _, _ = self[ind]
            if model:
                outputs, _ = model(torch.tensor(np.expand_dims(img, axis=0)).float())
                bbxs = find_batch_bounding_boxes(outputs)[0]
                img = util.draw_bounding_boxes(img, bbxs[Label.ROBOT.value], (0, 0, 255))
                img = util.draw_bounding_boxes(img, bbxs[Label.BALL.value], (255, 0, 0))
            else:
                bbxs = self.get_bounding_boxes(ind)
                img = util.draw_bounding_boxes(img, bbxs, 255)
            util.stream_image(img, delay, scale)

    def get_bounding_boxes(self, index):
        img_path = self.img_paths[index]
        img = util.read_image(img_path)

        # we need to scale bounding boxes since we applied a transformation
        height, width, _ = np.shape(img)
        height_scale = self.target_height / height
        width_offset = (width * height_scale - self.target_width) / 2
        bbxs = copy.deepcopy(self.bounding_boxes[img_path])
        for bbx in bbxs:
            bbx[0] = int(bbx[0] * height_scale - width_offset)
            bbx[1] = int(bbx[1] * height_scale)
            bbx[2] = int(bbx[2] * height_scale - width_offset)
            bbx[3] = int(bbx[3] * height_scale)
        return bbxs
