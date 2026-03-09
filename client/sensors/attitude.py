# -*- coding: UTF-8 -*-
"""
姿态角计算模块

基于加速度传感器计算姿态角(俯仰角和翻滚角)

参考文献:
张庆啟. 基于穿戴式设备的人体姿态识别研究[D]. 北京邮电大学, 2018.

姿态角计算公式:
- 俯仰角(pitch): 绕X轴旋转, 表示身体前后倾斜
- 翻滚角(roll): 绕Y轴旋转, 表示身体左右倾斜
"""

import math


class AttitudeCalculator:
    """姿态角计算器"""

    def __init__(self, config=None):
        if config is None:
            config = {
                "window_size": 10,
                "lowpass_alpha": 0.3,
            }

        self.config = config

        self.pitch_history = []
        self.roll_history = []

        self.filtered_pitch = 0
        self.filtered_roll = 0

    def calculate_angle(self, acc_x, acc_y, acc_z):
        """
        计算姿态角
        返回: (pitch, roll) - 俯仰角和翻滚角（度）

        - pitch (俯仰角): 绕X轴旋转，表示身体前后倾斜
        - roll (翻滚角): 绕Y轴旋转，表示身体左右倾斜
        """
        # 限制加速度值，防止除零错误
        ax = max(min(acc_x, 3.0), -3.0)
        ay = max(min(acc_y, 3.0), -3.0)
        az = max(min(acc_z, 3.0), -3.0)

        # 计算合量
        total = math.sqrt(ax**2 + ay**2 + az**2)
        if total < 0.1:
            return 0, 0

        # 俯仰角 (pitch): 绕X轴旋转 - 前后倾斜
        # 公式: pitch = atan2(ay, sqrt(ax^2 + az^2))
        pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi

        # 翻滚角 (roll): 绕Y轴旋转 - 左右倾斜
        # 公式: roll = atan2(ax, sqrt(ay^2 + az^2))
        roll = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi

        return pitch, roll

    def update(self, acc_x, acc_y, acc_z):
        """
        更新姿态角（带低通滤波）
        返回: (pitch, roll)
        """
        pitch, roll = self.calculate_angle(acc_x, acc_y, acc_z)

        # 低通滤波平滑
        alpha = self.config["lowpass_alpha"]
        self.filtered_pitch = alpha * pitch + (1 - alpha) * self.filtered_pitch
        self.filtered_roll = alpha * roll + (1 - alpha) * self.filtered_roll

        # 记录历史
        self.pitch_history.append(self.filtered_pitch)
        self.roll_history.append(self.filtered_roll)

        # 保持窗口大小
        max_history = self.config.get("window_size", 10)
        if len(self.pitch_history) > max_history:
            self.pitch_history.pop(0)
        if len(self.roll_history) > max_history:
            self.roll_history.pop(0)

        return self.filtered_pitch, self.filtered_roll

    def get_variance(self):
        """获取姿态角方差（用于运动检测）"""
        if len(self.pitch_history) < 2:
            return 0, 0

        # 俯仰角方差
        pitch_mean = sum(self.pitch_history) / len(self.pitch_history)
        pitch_var = sum((x - pitch_mean) ** 2 for x in self.pitch_history) / len(self.pitch_history)

        # 翻滚角方差
        roll_mean = sum(self.roll_history) / len(self.roll_history)
        roll_var = sum((x - roll_mean) ** 2 for x in self.roll_history) / len(self.roll_history)

        return math.sqrt(pitch_var), math.sqrt(roll_var)

    def get_filtered(self):
        """获取滤波后的姿态角"""
        return self.filtered_pitch, self.filtered_roll

    def reset(self):
        """重置历史数据"""
        self.pitch_history = []
        self.roll_history = []
        self.filtered_pitch = 0
        self.filtered_roll = 0
