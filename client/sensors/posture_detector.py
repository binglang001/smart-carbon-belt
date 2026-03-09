# -*- coding: UTF-8 -*-
"""
姿态检测模块

检测人体姿态(坐姿/站立/行走)

参考文献:
李毅. 基于加速度传感器的人体姿态识别研究[J]. 民间科技, 2022.
卢先领, 王洪斌, 王莹莹, 徐保国. 一种基于加速度传感器的人体跌倒识别方法[J]. 计算机应用研究, 2013, 30(4).

核心方法:
1. Roll角检测 - 基于绝对翻滚角判断坐/站
2. Y轴分量法 - 基于Y轴重力分量变化检测坐姿变化
3. 运动检测 - 区分静止和运动状态
"""

import math
from .attitude import AttitudeCalculator

# 导入日志模块
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.logger import get_logger
logger = get_logger('sensors.posture_detector')


class PostureDetector:
    """腰部佩戴的姿态检测器"""

    def __init__(self, config=None):
        if config is None:
            config = {
                "sit_pitch_change": 15,
                "sit_roll_change": 20,
                "motion_threshold": 0.05,
                "still_threshold": 0.02,
                "stable_window": 15,
                "hysteresis_count": 5,
                # Y轴分量法参数
                "y_baseline_samples": 30,
                "y_sit_threshold": 0.15,
                "y_stand_threshold": 0.10,
                # 姿态角计算参数
                "window_size": 10,
                "lowpass_alpha": 0.3,
            }

        self.config = config

        self.current_posture = "standing"
        self.stable_count = 0

        self.acc_history = []
        self.motion_window = 20

        # Y轴分量法参数
        self.y_baseline = None
        self.y_samples = []

        self.attitude = AttitudeCalculator(config)

    def update(self, acc_x, acc_y, acc_z):
        """更新姿态检测，返回: (是否变化, 当前姿态)"""
        pitch, roll = self.attitude.update(acc_x, acc_y, acc_z)

        # 计算当前加速度幅值
        acc_mag = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

        # 计算运动强度
        self.acc_history.append(acc_mag)
        if len(self.acc_history) > self.motion_window:
            self.acc_history.pop(0)

        motion_level = 0
        motion_detected = False
        if len(self.acc_history) >= 5:
            mean_acc = sum(self.acc_history) / len(self.acc_history)
            variance = sum((x - mean_acc) ** 2 for x in self.acc_history) / len(self.acc_history)
            std_acc = math.sqrt(variance)
            motion_level = std_acc
            # 运动检测：加速度变化超过阈值
            motion_detected = std_acc > self.config["motion_threshold"]

        # 优先使用Y轴分量法检测坐姿变化
        new_posture = self._determine_posture_by_y(acc_y, roll, motion_detected)

        # 如果Y轴方法返回None或"moving"，使用Pitch角方法作为后备
        if new_posture is None or new_posture == "moving":
            new_posture = self._determine_posture_by_pitch(pitch, motion_detected)

        changed = False
        if new_posture == self.current_posture:
            self.stable_count += 1
        else:
            # 需要连续多次检测到相同姿态才切换
            if self.stable_count >= self.config["hysteresis_count"]:
                changed = True
                logger.info(f"姿态切换: {self.current_posture} -> {new_posture} (pitch={pitch:.1f}°)")
                self.current_posture = new_posture
                self.stable_count = 0
            else:
                self.stable_count += 1

        return changed, self.current_posture

    def _determine_posture_by_y(self, acc_y, roll, motion_detected):
        """
        基于Y轴分量的坐姿检测

        原理：
        - 站立时：Y轴主要受重力影响，acc_y ≈ 0
        - 坐下时：Y轴重力分量会发生变化

        参数：
        - y_baseline: 站立时的Y轴基线值
        - y_sit_threshold: 坐下检测阈值
        - y_stand_threshold: 站立检测阈值
        """
        # 如果检测到明显运动，可能是行走/跑步
        if motion_detected:
            return "moving"

        # 收集基线样本
        if self.y_baseline is None:
            self.y_samples.append(acc_y)
            if len(self.y_samples) >= self.config["y_baseline_samples"]:
                # 取中位数作为基线，更稳定
                sorted_samples = sorted(self.y_samples)
                mid = len(sorted_samples) // 2
                self.y_baseline = sorted_samples[mid]
            return None  # 基线未建立完成

        # 计算当前Y轴与基线的差异
        delta_y = abs(acc_y - self.y_baseline)

        # 根据阈值判断
        if delta_y > self.config["y_sit_threshold"]:
            return "sitting"
        elif delta_y < self.config["y_stand_threshold"]:
            return "standing"

        # 在阈值之间时保持当前状态
        return None

    def _determine_posture_by_pitch(self, pitch, motion_detected):
        """
        基于Pitch角的姿态检测

        实际测试：
        - 站姿 pitch ≈ 89°
        - 坐姿 pitch ≈ 30-80°
        - 躺卧 pitch ≈ 0°
        """
        # 如果检测到明显运动，可能是行走/跑步
        if motion_detected:
            return "moving"

        # 根据pitch角判断
        if pitch > 80:
            return "standing"
        elif pitch < 30:
            return "lying"
        else:
            # 30到70之间为坐姿
            return "sitting"

    def get_posture(self):
        """获取当前姿态"""
        return self.current_posture

    def get_attitude(self):
        """获取当前姿态角"""
        return self.attitude.get_filtered()

    def get_motion_level(self):
        """获取运动强度"""
        if len(self.acc_history) < 5:
            return 0
        mean_acc = sum(self.acc_history) / len(self.acc_history)
        variance = sum((x - mean_acc) ** 2 for x in self.acc_history) / len(self.acc_history)
        return math.sqrt(variance)

    def reset(self):
        """重置检测器"""
        self.current_posture = "standing"
        self.stable_count = 0
        self.acc_history = []
        self.y_baseline = None
        self.y_samples = []
