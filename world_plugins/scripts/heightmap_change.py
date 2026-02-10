"""Heightmap augmentation utilities: noise injection and synthetic edits."""

import cv2
import numpy as np
import os
import random
from pathlib import Path


def noise(img, snr):
    """Apply salt-and-pepper noise to a grayscale image.

    Args:
        img (np.ndarray): Input uint16 heightmap.
        snr (float): Signal-to-noise ratio (0..1).
    Returns:
        np.ndarray: Noisy image.
    """
    h = img.shape[0]
    w = img.shape[1]
    img1 = img.copy()
    sp = h * w  # 计算图像像素点个数
    NP = int(sp * (1 - snr))  # 计算图像椒盐噪声点个数
    for i in range(NP):
        randx = np.random.randint(1, h - 1)  # 生成一个 1 至 h-1 之间的随机整数
        randy = np.random.randint(1, w - 1)  # 生成一个 1 至 w-1 之间的随机整数
        if (
            np.random.random() <= 0.5
        ):  # np.random.random()生成一个 0 至 1 之间的浮点数
            img1[randx, randy] = 0
        else:
            img1[randx, randy] = 2**16 - 1
    return img1


def noise_label(img, label, snr):
    """Apply salt-and-pepper noise to labeled regions only."""
    h = img.shape[0]
    w = img.shape[1]
    img1 = img.copy()
    sp = h * w  # 计算图像像素点个数
    NP = int(sp * (1 - snr))  # 计算图像椒盐噪声点个数
    for i in range(NP):
        randx = np.random.randint(1, h - 1)  # 生成一个 1 至 h-1 之间的随机整数
        randy = np.random.randint(1, w - 1)  # 生成一个 1 至 w-1 之间的随机整数
        if label[randx, randy] == 1:
            if (
                np.random.random() <= 0.5
            ):  # np.random.random()生成一个 0 至 1 之间的浮点数
                img1[randx, randy] = int(2**16 * 0.8)
            else:
                img1[randx, randy] = 2**16 - 1
    return img1


def gen_heightmap(img_num):
    """Generate a new heightmap variant from an existing source.

    Args:
        img_num (int): Source heightmap index (HM{img_num}.png).
    """
    x_min = int((-4.2 + 5) * 640 / 10)
    x_max = int((4.2 + 5) * 640 / 10)
    y_min = int((-2.2 + 5) * 640 / 10)
    y_max = int((2.2 + 5) * 640 / 10)

    l_min = int(0.8 / 10 * 640)

    repo_root = Path(__file__).resolve().parents[2]  # .../MarsSim
    file_path_16 = repo_root / "rover_gazebo" / "models" / "mars_terrain" / "choose" / "heightmaps_int16"
    file_path_8 = repo_root / "rover_gazebo" / "models" / "mars_terrain" / "choose" / "heightmaps_int8"

    img = cv2.imread(str(file_path_16 / f"HM{img_num}.png"), -1)
    ii = random.randint(0, 3)
    for i in range(ii):
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    img_cp = (img.copy() * 0.85).astype('uint16')
    cv2.imwrite(str(file_path_16 / "HM9_o.png"), img_cp)
    img_cp_o = cv2.resize(img_cp.copy(), (513, 513))
    img_cp_o = (img_cp_o // 256).astype('uint8')
    cv2.imwrite(str(file_path_8 / "HM9_o.png"), img_cp_o)

    label = np.zeros((640, 640), dtype=np.uint8)
    num = random.randint(1, 3)
    for i in range(num):
        cc = random.randint(0, 1)
        if cc == 0:
            x_s = random.randint(x_min, x_max - l_min)
            y_s = random.randint(y_min, y_max - l_min)
            x_e = random.randint(x_s + l_min, x_max)
            y_e = random.randint(y_s + l_min, y_max)
            if x_e - x_s > 128:
                x_e = x_s + 128
            if y_e - y_s > 128:
                y_e = y_s + 128
            label = cv2.rectangle(label, (x_s, y_s), (x_e, y_e), (1), -1)
        else:
            x_c = random.randint(x_min + l_min, x_max - l_min)
            y_c = random.randint(y_min + l_min, y_max - l_min)
            l = random.randint(int(l_min * 0.75), int(l_min * 1.25))
            axes_x = int((random.random() + 1) * l)
            axes_y = l
            a = random.random() * 180
            label = cv2.ellipse(
                img=label,
                center=(x_c, y_c),
                axes=(axes_x, axes_y),
                angle=a,
                startAngle=0,
                endAngle=360,
                color=(1),
                thickness=-1,
            )

    img_ = np.zeros((640, 640), dtype=np.uint16)
    img_[label == 1] = int(0.92 * 2**16 - 1)
    img__ = noise(img_, 0.75)
    img_cp[label == 1] = img__[label == 1]
    img_cp = cv2.GaussianBlur(img_cp, (3, 3), 1)

    img_cp_ = img_cp.copy()
    img_cp_ = cv2.GaussianBlur(img_cp_, (37, 37), 23)

    img_cp_ = (
        (img_cp_ - np.min(img_cp_))
        * ((2**16 - 1) / (np.max(img_cp_) - np.min(img_cp_)))
    ).astype('uint16')
    # img_end = noise_label(img_cp_, label, 0.8)
    # img_end = cv2.GaussianBlur(img_end, (5, 5), 2)
    img_end = img_cp_
    cv2.imwrite(str(file_path_16 / "HM9.png"), img_end)

    img_end = cv2.resize(img_end, (513, 513))
    img_end = (img_end // 256).astype('uint8')
    cv2.imwrite(str(file_path_8 / "HM9.png"), img_end)
    # cv2.imshow('img', img_cp_)
    # cv2.waitKey()


if __name__ == "__main__":
    gen_heightmap(8)
