"""Procedural geometry generator for terrain class maps and textures."""

""""""
import cv2
import numpy as np
import random
from pathlib import Path


def label_colormap(N=256):
    """Generate a Pascal VOC-style label colormap.

    Args:
        N (int): Number of labels/colors.
    Returns:
        np.ndarray: Colormap of shape (N, 3) in uint8.
    """

    def bitget(byteval, idx):
        return (byteval & (1 << idx)) != 0

    cmap = np.zeros((N, 3))
    for i in range(0, N):
        id = i
        r, g, b = 0, 0, 0
        for j in range(0, 8):
            r = np.bitwise_or(r, (bitget(id, 0) << 7 - j))
            g = np.bitwise_or(g, (bitget(id, 1) << 7 - j))
            b = np.bitwise_or(b, (bitget(id, 2) << 7 - j))
            id = id >> 3
        cmap[i, 0] = r
        cmap[i, 1] = g
        cmap[i, 2] = b
    # cmap = cmap.astype(np.float32) / 255
    return cmap


def _resolve_output_dir(save_base_path=None) -> Path:
    """Resolve the output directory for whole texture assets.

    If save_base_path is None/empty, infer:
      <MarsSim>/rover_gazebo/models/mars_terrain/whole_tex
    based on this file location.

    Args:
        save_base_path (str|Path|None): Base directory to write into.
            If provided, outputs will be written to:
              <save_base_path>/rover_gazebo/models/mars_terrain/whole_tex

    Returns:
        Path: Directory that should contain terrain_tex.png and terrain_class.npy
    """
    if save_base_path is not None and str(save_base_path).strip() != "":
        base = Path(save_base_path).expanduser().resolve()
    else:
        # plot_geometry.py: .../MarsSim/world_plugins/scripts/plot_geometry.py
        # parents[2] -> .../MarsSim
        base = Path(__file__).resolve().parents[2]

    out_dir = base / "rover_gazebo" / "models" / "mars_terrain" / "whole_tex"
    return out_dir


def generate_class_mat(save_base_path=None):
    """Create a random semantic class map and matching texture image.

    Writes:
        - terrain_tex.png: synthetic texture
        - terrain_class.npy: class matrix (labels)

    Args:
        save_base_path (str|Path|None): Optional base path override.
            See _resolve_output_dir().
    Raises:
        ValueError: If save_base_path is provided but invalid.
        RuntimeError: If output directory can't be created or files can't be written.
    """
    # ---- exit/validation mechanism ----
    try:
        out_dir = _resolve_output_dir(save_base_path)
    except Exception as e:
        raise ValueError(f"Invalid save_base_path={save_base_path!r}") from e

    try:
        out_dir.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        raise RuntimeError(
            f"Failed to create output directory: {out_dir}"
        ) from e

    tex_path = out_dir / "terrain_tex.png"
    class_path = out_dir / "terrain_class.npy"

    color_list = label_colormap(10)[1:]
    h = 1024
    img = np.zeros((h, h, 3), np.uint8)
    class_mat = np.zeros((641, 641), np.uint8)
    rate = 641 / h
    img[:, :, 0] = 128
    num = 50
    for i in range(num):
        # 选择形状
        c = random.randint(1, 2)
        # 选择颜色
        n = random.randint(1, 3)
        color = color_list[n]
        # 选择参数
        if c == 1:
            # 圆形
            center = (random.randint(0, h - 1), random.randint(0, h - 1))
            radius = random.randint(20, 128)
            img = cv2.circle(img, center, radius, color, -1)
            class_mat = cv2.circle(
                class_mat,
                (int(center[0] * rate), int(center[1] * rate)),
                int(radius * rate),
                n,
                -1,
            )
        elif c == 2:
            # 矩形
            r_1 = random.randint(50, 250)
            r_2 = random.randint(50, 250)
            pt1 = (random.randint(0, h - 1), random.randint(0, h - 1))
            pt2 = (pt1[0] + r_1, pt1[1] + r_2)
            img = cv2.rectangle(img, pt1, pt2, color, -1)
            class_mat = cv2.rectangle(
                class_mat,
                (int(pt1[0] * rate), int(pt1[1] * rate)),
                (int(pt2[0] * rate), int(pt2[1] * rate)),
                n,
                -1,
            )
    mean = 0
    # 设置高斯分布的标准差
    sigma = 25
    # 根据均值和标准差生成符合高斯分布的噪声
    gauss = np.random.normal(mean, sigma, (h, h, 3))
    # 给图片添加高斯噪声
    noisy_img = img + gauss
    # 设置图片添加高斯噪声之后的像素值的范围
    img = np.clip(noisy_img, a_min=0, a_max=255)

    ok = cv2.imwrite(str(tex_path), img)
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed for: {tex_path}")

    try:
        np.save(str(class_path), class_mat)
    except Exception as e:
        raise RuntimeError(f"np.save failed for: {class_path}") from e

    return str(tex_path), str(class_path)


if __name__ == "__main__":
    # Keep default behavior (no args) for compatibility with ModelGEN.py
    generate_class_mat()
