# -*- coding: UTF-8 -*-
import math
import shutil
import os
from pathlib import Path
from typing import Union

def quaternion_from_euler(r, p, y):
    '''欧拉角转四元数'''
    q = [0, 0, 0, 0]
    q[3] = math.cos(r / 2) * math.cos(p / 2) * math.cos(y / 2) + math.sin(
        r / 2
    ) * math.sin(p / 2) * math.sin(y / 2)
    q[0] = math.sin(r / 2) * math.cos(p / 2) * math.cos(y / 2) - math.cos(
        r / 2
    ) * math.sin(p / 2) * math.sin(y / 2)
    q[1] = math.cos(r / 2) * math.sin(p / 2) * math.cos(y / 2) + math.sin(
        r / 2
    ) * math.cos(p / 2) * math.sin(y / 2)
    q[2] = math.cos(r / 2) * math.cos(p / 2) * math.sin(y / 2) - math.sin(
        r / 2
    ) * math.sin(p / 2) * math.cos(y / 2)
    return q


def delete_paging(paging_dir: Union[str, os.PathLike, None]=None):
    '''删除gazebo paging文件夹

    Args:
        paging_dir: 要删除的 paging 目录。
            - None: 默认删除 ~/.gazebo/paging(不再硬编码用户名)
            - str/PathLike: 支持相对路径(相对于当前工作目录)，内部会 resolve
    '''
    if paging_dir is None:
        paging_path = Path.home() / ".gazebo" / "paging"
    else:
        paging_path = Path(paging_dir).expanduser().resolve()

    if paging_path.exists():
        shutil.rmtree(paging_path)


from omegaconf import DictConfig, OmegaConf
from typing import Dict


def omegaconf_to_dict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation."""
    ret = {}
    for k, v in d.items():
        if isinstance(v, DictConfig):
            ret[k] = omegaconf_to_dict(v)
        else:
            ret[k] = v
    return ret


def print_dict(val, nesting: int = -4, start: bool = True):
    """Outputs a nested dictionory."""
    if type(val) == dict:
        if not start:
            print('')
        nesting += 4
        for k in val:
            print(nesting * ' ', end='')
            print(k, end=': ')
            print_dict(val[k], nesting, start=False)
    else:
        print(val)
