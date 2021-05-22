import numpy as np
import PIL
import cv2
import matplotlib.pyplot as plt


def torch_to_cv(img):
    if isinstance(img, np.ndarray):
        return np.moveaxis(img, 0, -1)

    img = np.array(img.detach().cpu())
    if len(img.shape) >= 3:
        assert img.shape[0] <= 3  # first dim is color channel
        return np.moveaxis(img, 0, -1)
    else:
        assert len(img.shape) == 2  # gray scale
        return np.squeeze(img)


def cv_to_torch(img):
    assert img.shape[2]  # last dim is color channel
    return np.moveaxis(img, 2, 0)


def draw_bounding_boxes(img, bbxs, colour):
    img = torch_to_cv(img)
    img = img.copy()  # cv2 seems to like copies to draw rectangles on

    for bbx in bbxs:
        pt0 = (int(bbx[0]), int(bbx[1]))
        pt1 = (int(bbx[2]), int(bbx[3]))
        img = cv2.rectangle(img, pt0, pt1, colour, 1)
    return cv_to_torch(img)


def display_image(to_plot):
    '''
    :param to_plot: list of tuples of the form (img [(cxhxw) numpy array], cmap [str], title [str])
    '''
    fig, ax = plt.subplots(3, 2, figsize=(8, 10))
    for i, plot_info in enumerate(to_plot):
        img = torch_to_cv(plot_info[0])
        cmap = plot_info[1]
        title = plot_info[2]

        ax[i // 2, i % 2].imshow(img, cmap=cmap)
        ax[i // 2, i % 2].set_title(title)
    plt.show()


def stream_image(img, wait, scale):
    img = torch_to_cv(img)
    width, height, _ = img.shape
    img = cv2.resize(img, (height * scale, width * scale))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow('my_window', img)
    cv2.waitKey(wait)


def read_image(path):
    # using opencv imread crashes Pytorch DataLoader for some reason
    return PIL.Image.open(path)


def subset_label_count(subset, ball, robot):
    ball_count = 0
    robot_count = 0
    for ind in subset.indices:
        img_path = subset.dataset.img_paths[ind]
        bbxs = subset.dataset.bounding_boxes[img_path]
        for bbx in bbxs:
            if bbx[4] == ball:
                ball_count += 1
            elif bbx[4] == robot:
                robot_count += 1
            else:
                print('subset_label_count PROBLEM!')
    return ball_count, robot_count
