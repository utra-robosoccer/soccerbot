import torch
import torch.nn as nn
import cv2
import numpy as np
import enum
import util


class Label(enum.Enum):
    # Defines output channels of model
    BALL = 0
    ROBOT = 1
    OTHER = 2


class CNN(nn.Module):
    """
    Model reproduced from: Hamburg Bit Bot Team
    assume input image has dimensions 3x150x200
    """

    def __init__(self, kernel=3, num_features=16, dropout=0.5):
        super(CNN, self).__init__()

        pad = kernel // 2  # ensure output will have the same dimensions as input

        self.conv1 = nn.Sequential(
            nn.Conv2d(3, num_features, kernel, padding=pad),
            nn.BatchNorm2d(num_features)
        )

        self.maxpool1 = nn.Sequential(
            nn.LeakyReLU(),
            nn.MaxPool2d(2)
        )

        self.conv2 = nn.Sequential(
            nn.Conv2d(num_features, 2 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(2 * num_features),
            nn.LeakyReLU()
        )

        self.conv3 = nn.Sequential(
            nn.Conv2d(2 * num_features, 2 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(2 * num_features),
            nn.LeakyReLU()
        )

        self.maxpool2 = nn.Sequential(
            nn.MaxPool2d(2)
        )

        # concat 16 + 32
        self.conv4 = nn.Sequential(
            nn.Conv2d(3 * num_features, 4 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(4 * num_features),
            nn.LeakyReLU()
        )

        self.conv5 = nn.Sequential(
            nn.Conv2d(4 * num_features, 4 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(4 * num_features),
            nn.LeakyReLU()
        )

        # concat 48 + 32
        self.conv6 = nn.Sequential(
            nn.Conv2d(7 * num_features, 8 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(8 * num_features),
            nn.LeakyReLU()
        )

        self.conv7 = nn.Sequential(
            nn.Conv2d(8 * num_features, 8 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(8 * num_features),
            nn.LeakyReLU(),
            nn.UpsamplingBilinear2d(size=(75, 100))  # assume input was 250x200pxl
        )

        # concat 48 + 128
        self.conv8 = nn.Sequential(
            nn.Conv2d(11 * num_features, 4 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(4 * num_features),
            nn.LeakyReLU()
        )

        self.conv9 = nn.Sequential(
            nn.Conv2d(4 * num_features, 2 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(2 * num_features),
            nn.LeakyReLU()
        )

        self.conv10 = nn.Sequential(
            nn.Conv2d(2 * num_features, 2 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(2 * num_features),
            nn.LeakyReLU(),
            nn.UpsamplingBilinear2d(size=(150, 200))  # assume input was 150x200pxl
        )

        # concat 16 + 32
        self.conv11 = nn.Sequential(
            nn.Conv2d(3 * num_features, 1 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(1 * num_features),
            nn.LeakyReLU()
        )

        self.conv12 = nn.Sequential(
            nn.Conv2d(1 * num_features, 1 * num_features, kernel, padding=pad),
            nn.Dropout2d(p=dropout),
            nn.BatchNorm2d(1 * num_features),
            nn.LeakyReLU()
        )

        self.conv13 = nn.Sequential(
            nn.Conv2d(1 * num_features, 3, kernel, padding=pad)
        )

    def forward(self, x):
        conv1_out = self.conv1(x)
        max1_out = self.maxpool1(conv1_out)
        x = self.conv2(max1_out)
        x = self.conv3(x)

        cat1 = torch.cat((max1_out, x), 1)
        max2 = self.maxpool2(cat1)
        x = self.conv4(max2)
        x = self.conv5(x)
        x = torch.cat((max2, x), 1)
        x = self.conv6(x)

        # Decoding part
        x = self.conv7(x)

        cat2 = torch.cat((cat1, x), 1)
        x = self.conv8(cat2)
        x = self.conv9(x)
        x = self.conv10(x)

        x = torch.cat((conv1_out, x), 1)
        x = self.conv11(x)
        x = self.conv12(x)
        logit = self.conv13(x)  # used for calculating loss
        output = torch.nn.Softmax2d()(logit)

        return output, logit


def init_weights(m):
    if type(m) == nn.Conv2d:
        torch.nn.init.uniform_(m.weight, a=-0.001, b=0.001)
        m.bias.data.fill_(0.00)


def find_batch_bounding_boxes(outputs):
    """find bounding boxes for batch of outputs from the model
    :return 3 dimensional list [batch][class prediction][bounding box]"""
    batch_bbxs = []

    for output in outputs:
        output_bbxs = [[], []]

        for label in [Label.BALL, Label.ROBOT]:
            img = output[label.value]
            output_bbxs[label.value] = find_bounding_boxes(img)

        batch_bbxs.append(output_bbxs)

    return batch_bbxs


def find_bounding_boxes(img):
    """
    Find bounding boxes for blobs in the picture
    :param img - numpy array 1xWxH, values 0 to 1
    :return:  bounding boxes of blobs [x0, y0, x1, y1]
    """
    img = util.torch_to_cv(img)
    img = np.round(img)
    img = img.astype(np.uint8)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # params copied from tutorial

    bounding_boxes = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        bounding_boxes.append([x, y, x + w, y + h])

    return bounding_boxes
