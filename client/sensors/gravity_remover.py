# -*- coding: UTF-8 -*-
"""
重力加速度去除模块

参考文献:
胡成全, 王凯, 何丽莉等. 基于MEMS六轴传感器的上肢运动识别系统[J]. 吉林大学学报(信息科学版), 2018.

核心算法:
1. 一阶低通滤波器分离重力加速度和线性加速度
2. 滑动平均滤波器去噪

算法公式:
Ag(n) = α × Ad(n) + (1-α) × Ag(n-1)
Al(n) = Ad(n) - Ag(n)

其中:
- Ad: 传感器测量加速度
- Ag: 重力加速度
- Al: 线性加速度(运动产生)
- α: 滤波系数(默认0.3)
"""

import math
from collections import deque

# 导入日志模块
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.logger import get_logger

# 导入config获取debug设置
import config

# 获取日志器
logger = get_logger('sensors.gravity_remover')

# Debug开关
DEBUG_GRAVITY = config.DEBUG_ENABLED


class GravityRemover:
    """
    基于低通滤波的重力去除器
    """

    def __init__(self, config=None):
        if config is None:
            # 从全局config读取参数
            try:
                import config as config_module
                config = config_module.GRAVITY_REMOVER_CONFIG
            except:
                config = {
                    "filter_alpha": 0.3,
                    "filter_window": 5,
                }

        self.config = config
        self.filter_alpha = config.get("filter_alpha", 0.3)
        self.filter_window = config.get("filter_window", 5)

        # 重力加速度初始值
        self.gravity_x = 0.0
        self.gravity_y = 0.0
        self.gravity_z = 0.0

        # 是否首次采样
        self.is_first_sample = True

        # 滑动平均滤波器缓冲区
        self.filter_buffer_x = deque(maxlen=self.filter_window)
        self.filter_buffer_y = deque(maxlen=self.filter_window)
        self.filter_buffer_z = deque(maxlen=self.filter_window)

        # 调试统计
        self.sample_count = 0

    def add_sample(self, acc_x, acc_y, acc_z, gyro_x=None, gyro_y=None, gyro_z=None, timestamp=None):
        """
        添加样本并进行重力去除

        参数:
            acc_x, acc_y, acc_z: 三轴加速度 (g)
            gyro_x, gyro_y, gyro_z: 三轴角速度 (°/s)，可选
            timestamp: 时间戳

        返回:
            (linear_x, linear_y, linear_z) - 去除重力后的线性加速度 (g)
        """
        self.sample_count += 1

        if self.is_first_sample:
            # 首次采样：用原始加速度初始化重力加速度
            # 这样第一个样本的线性加速度接近0（重力被正确估计）
            self.gravity_x = acc_x
            self.gravity_y = acc_y
            self.gravity_z = acc_z

            self.filter_buffer_x.clear()
            self.filter_buffer_y.clear()
            self.filter_buffer_z.clear()

            linear_x = 0.0
            linear_y = 0.0
            linear_z = 0.0

            self.is_first_sample = False
        else:
            # 论文公式(4): Ag(n) = α × Ad(n) + (1-α) × Ag(n-1)
            self.gravity_x = self.filter_alpha * acc_x + (1 - self.filter_alpha) * self.gravity_x
            self.gravity_y = self.filter_alpha * acc_y + (1 - self.filter_alpha) * self.gravity_y
            self.gravity_z = self.filter_alpha * acc_z + (1 - self.filter_alpha) * self.gravity_z

            # 论文公式(5): Al(n) = Ad(n) - Ag(n)
            linear_x = acc_x - self.gravity_x
            linear_y = acc_y - self.gravity_y
            linear_z = acc_z - self.gravity_z

        # 添加到滑动平均滤波器缓冲区
        self.filter_buffer_x.append(linear_x)
        self.filter_buffer_y.append(linear_y)
        self.filter_buffer_z.append(linear_z)

        # 应用滑动平均滤波
        if len(self.filter_buffer_x) >= self.filter_window:
            linear_x = sum(self.filter_buffer_x) / len(self.filter_buffer_x)
            linear_y = sum(self.filter_buffer_y) / len(self.filter_buffer_y)
            linear_z = sum(self.filter_buffer_z) / len(self.filter_buffer_z)

        if DEBUG_GRAVITY and self.sample_count <= 5:
            logger.debug(f"[重力去除] 样本{self.sample_count}: "
                        f"原始=({acc_x:.4f}, {acc_y:.4f}, {acc_z:.4f}), "
                        f"重力=({self.gravity_x:.4f}, {self.gravity_y:.4f}, {self.gravity_z:.4f}), "
                        f"线性=({linear_x:.4f}, {linear_y:.4f}, {linear_z:.4f})")

        return linear_x, linear_y, linear_z

    def get_gravity(self):
        """获取当前重力加速度估计值"""
        return self.gravity_x, self.gravity_y, self.gravity_z

    def reset(self):
        """重置状态"""
        self.gravity_x = 0.0
        self.gravity_y = 0.0
        self.gravity_z = 0.0
        self.is_first_sample = True
        self.filter_buffer_x.clear()
        self.filter_buffer_y.clear()
        self.filter_buffer_z.clear()
        self.sample_count = 0

    def set_parameters(self, alpha=None, window=None):
        """
        设置参数

        参数:
            alpha: 滤波系数 (0-1)
            window: 滑动平均窗口大小
        """
        if alpha is not None:
            self.filter_alpha = alpha
            logger.info(f"滤波系数已更新: {alpha}")

        if window is not None:
            self.filter_window = window
            # 重建缓冲区
            self.filter_buffer_x = deque(maxlen=window)
            self.filter_buffer_y = deque(maxlen=window)
            self.filter_buffer_z = deque(maxlen=window)
            logger.info(f"滑动窗口已更新: {window}")


# 全局单例
_gravity_remover_instance = None


def get_gravity_remover():
    """获取全局重力去除器实例"""
    global _gravity_remover_instance
    if _gravity_remover_instance is None:
        _gravity_remover_instance = GravityRemover()
    return _gravity_remover_instance


def remove_gravity(acc_x, acc_y, acc_z, gyro_x=None, gyro_y=None, gyro_z=None, timestamp=None):
    """
    去除重力加速度

    参数:
        acc_x, acc_y, acc_z: 三轴加速度 (g)
        gyro_x, gyro_y, gyro_z: 三轴角速度 (°/s)，可选
        timestamp: 时间戳

    返回:
        (linear_x, linear_y, linear_z) - 去除重力后的线性加速度 (g)
    """
    remover = get_gravity_remover()
    return remover.add_sample(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp)
