import os
import time
import enum
import numpy as np
import torch
from model import Label, find_batch_bounding_boxes
from my_dataset import initialize_loader
from util import display_image, draw_bounding_boxes
import matplotlib.pyplot as plt


class Trainer:
    '''Assume model is trained on GPU enabled computer'''

    class ErrorType(enum.Enum):
        TRUE_POSITIVE = 0
        FALSE_POSITIVE = 1
        TRUE_NEGATIVE = 2
        FALSE_NEGATIVE = 3

    def __init__(self, model, learn_rate, weight_decay, batch_size, epochs, class_weights, colour_jitter, seed,
                 output_folder):
        self.model = model
        self.learn_rate = learn_rate
        self.batch_size = batch_size
        self.epochs = epochs
        self.output_folder = output_folder
        self.seed = seed
        self.optimizer = torch.optim.Adam(model.parameters(), lr=learn_rate, weight_decay=weight_decay)
        self.class_weights = torch.tensor(class_weights)  # weigh importance of the label during training
        self.criterion = torch.nn.CrossEntropyLoss(weight=self.class_weights.cuda())

        torch.manual_seed(seed)
        np.random.seed(seed)

        self.train_losses = []
        self.train_ious = []
        self.train_radius_losses = []

        self.valid_losses = []
        self.valid_ious = []
        self.valid_radius_losses = []

        self.valid_stats = {'BALL': [], 'ROBOT': []}
        self.precision = 0
        self.recall = 0

        loaders, datasets = initialize_loader(batch_size, jitter=colour_jitter)
        self.train_loader, self.valid_loader, self.test_loader = loaders
        self.train_dataset, self.test_dataset = datasets
        print('Datasets Loaded! # of batches train:{} valid:{} test:{}'.format(
            len(self.train_loader), len(self.valid_loader), len(self.test_loader)))

    def train(self):
        print('Starting Training')
        start_train = time.time()

        self.model.cuda()
        for epoch in range(self.epochs):
            self.train_epoch(epoch)
            self.test_model('valid', epoch)

        self.test_model('test', 'test')

        time_elapsed = time.time() - start_train
        print('Finished training in: {: 4.2f}min'.format(time_elapsed / 60))

        self.plot_losses()

    def train_epoch(self, epoch):
        self.model.train()
        start_epoch = time.time()
        batchload_times = []
        losses = []
        t_readimg = time.time()
        for images, masks, img_paths in self.train_loader:
            batchload_times.append(time.time() - t_readimg)

            images, masks = images.cuda(), masks.cuda()

            self.optimizer.zero_grad()
            _, logits = self.model(images.float())
            loss = self.criterion(logits, masks.long())
            loss.backward()
            self.optimizer.step()
            losses.append(loss.data.item())

            t_readimg = time.time()
        self.train_losses.append(sum(losses) / len(losses))

        time_elapsed = time.time() - start_epoch
        print('Epoch [{:2d}/{:2d}]: Train Loss: {: 4.6f}, Avg. Batch Load (s): {:.4f}, Epoch (s): {: 4.2f}'.format(
            epoch + 1,
            self.epochs,
            self.train_losses[-1],
            sum(batchload_times) / len(batchload_times),
            time_elapsed))

    def test_model(self, test_type, epoch):
        dataset, loader = None, None
        if test_type == 'valid':
            dataset, loader = self.train_dataset, self.valid_loader
        elif test_type == 'test':
            dataset, loader = self.test_dataset, self.test_loader

        self.model.eval()
        start_valid = time.time()
        losses = []
        stats = {Label.BALL: [0, 0, 0, 0], Label.ROBOT: [0, 0, 0, 0]}
        for images, masks, indexes in loader:
            images = images.cuda()
            masks = masks.cuda()
            outputs, logits = self.model(images.float())
            loss = self.criterion(logits, masks.long())
            losses.append(loss.data.item())

            bbxs = find_batch_bounding_boxes(outputs)
            self.update_batch_stats(stats, bbxs, masks, dataset, indexes)

        # Show sample image with bounding boxes to get feel for what model is learning
        for i in range(1):
            img = draw_bounding_boxes(images[i], bbxs[i][Label.BALL.value], (255, 0, 0))  # balls
            img = draw_bounding_boxes(img, bbxs[i][Label.ROBOT.value], (0, 0, 255))  # robots

            display_image([
                (img, None, 'Epoch: ' + str(epoch)),
                (masks[i], None, 'Truth'),
                (outputs[i], None, 'Prediction'),
                (outputs[i][Label.OTHER.value], 'gray', 'Background'),
                (outputs[i][Label.BALL.value], 'gray', 'Ball'),
                (outputs[i][Label.ROBOT.value], 'gray', 'Robot')
            ])

        self.valid_losses.append(np.sum(losses) / len(losses))
        time_elapsed = time.time() - start_valid

        print('{:>20} Loss: {: 4.6f}, , {} time (s): {: 4.2f}'.format(
            test_type,
            self.valid_losses[-1],
            test_type,
            time_elapsed))

        for label in [Label.BALL, Label.ROBOT]:
            total = {}  # number of labels
            if test_type == 'test':
                total[Label.BALL] = dataset.num_ball_labels
                total[Label.ROBOT] = dataset.num_robot_labels
            elif test_type == 'valid':
                total[Label.BALL] = dataset.num_ball_labels - dataset.num_train_ball_labels
                total[Label.ROBOT] = dataset.num_robot_labels - dataset.num_train_robot_labels

            tp = stats[label][self.ErrorType.TRUE_POSITIVE.value]
            fp = stats[label][self.ErrorType.FALSE_POSITIVE.value]
            tn = stats[label][self.ErrorType.TRUE_NEGATIVE.value]
            fn = total[label] - tp  # proxy approximation for fn

            self.precision = 0.0
            if tp + fp > 0:
                self.precision = tp / (tp + fp)
            self.recall = tp / (tp + fn)
            print('{:>20} {} tp:{:6d}, fp:{:6d}, tn:{:6d}, proxy_fn:{:6d}, ' \
                  'precision:{:.4f}, recall:{:.4f}, total {}'.format(
                '', label.name, tp, fp, tn, fn,
                self.precision, self.recall, total[label]
            ))

    def update_batch_stats(self, stats, batch_bounding_boxes, batch_masks, dataset, batch_img_indexes):
        """
        given predicted bounding boxes and ground truth masks calculate true positive/negative
        define successful prediction as: the center of bounding box falls on the correct mask
        false positive/negative is not calculated as it is hard to define in multi object/label picture
        """
        for batch_ind, bounding_boxes in enumerate(batch_bounding_boxes):
            mask = batch_masks[batch_ind]
            for pred_class in [Label.BALL, Label.ROBOT]:
                for bbx in bounding_boxes[pred_class.value]:
                    x_center = int((bbx[0] + bbx[2]) / 2)
                    y_center = int((bbx[1] + bbx[3]) / 2)
                    if mask[y_center][x_center] == pred_class.value:
                        stats[pred_class][self.ErrorType.TRUE_POSITIVE.value] += 1
                    else:
                        stats[pred_class][self.ErrorType.FALSE_POSITIVE.value] += 1


    def training_name(self):
        name = 'lr{:.6f}_bs{}_ep{}_w{:.2f}-{:.2f}-{:.2f}'.format(
            self.learn_rate,
            self.batch_size,
            self.epochs,
            self.class_weights[0],
            self.class_weights[1],
            self.class_weights[2]
        )
        return name

    def plot_losses(self):
        plt.figure()
        plt.ylim(0.0, 0.2)
        plt.grid(True)
        plt.plot(self.train_losses, "ro-", label="Train")
        plt.plot(self.valid_losses, "go-", label="Validation")
        plt.legend()
        plt.title("Losses")
        plt.xlabel("Epochs")
        plt.savefig(os.path.join(self.output_folder, self.training_name() + ".png"))
        plt.show()
