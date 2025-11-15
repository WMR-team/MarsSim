# -*- coding: UTF-8 -*-
import math
import shutil
import os
from pathlib import Path

def quaternion_from_euler(r, p, y):
    '''欧拉角转四元数'''
    q = [0, 0, 0, 0]
    q[3] = math.cos(r / 2) * math.cos(p / 2) * math.cos(y / 2) + \
        math.sin(r / 2) * math.sin(p / 2) * math.sin(y / 2)
    q[0] = math.sin(r / 2) * math.cos(p / 2) * math.cos(y / 2) - \
        math.cos(r / 2) * math.sin(p / 2) * math.sin(y / 2)
    q[1] = math.cos(r / 2) * math.sin(p / 2) * math.cos(y / 2) + \
        math.sin(r / 2) * math.cos(p / 2) * math.sin(y / 2)
    q[2] = math.cos(r / 2) * math.cos(p / 2) * math.sin(y / 2) - \
        math.sin(r / 2) * math.sin(p / 2) * math.cos(y / 2)
    return q

def delete_paging():
    '''删除gazebo paging文件夹'''
    paging_dir = Path.home() / ".gazebo" / "paging"
    if paging_dir.exists():
        shutil.rmtree(paging_dir)
