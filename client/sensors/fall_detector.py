# -*- coding: UTF-8 -*-
"""
跌倒检测模块

参考文献:
李争. 一种基于三轴加速度的跌倒检测方法[J]. 电子科技, 2015, 28(3).

论文算法描述:
1. 合加速度: RA = √(ax² + ay² + az²)
2. 加速度变化率: SA = (1/t) × [∫Ax(t)dt + ∫Ay(t)dt + ∫Az(t)dt]
3. 加速度能量: E = ∫s(t)²dt = ∫[Ax(t)² + Ay(t)² + Az(t)²]dt
4. 方向变化: D = √[Σ(xi - yi)²], 其中x为当前帧, y为上一帧

跌倒判定: RA > 1.85g, SA > 2.2, E > 3.5, Dip > 0.3 同时满足
"""

import math
from collections import deque


class FallDetector:
    """基于三轴加速度跌倒检测方法的跌倒检测器"""

    def __init__(self, config=None):
        if config is None:
            config = {
                "ra_threshold": 1.85,      # 合加速度阈值 (g)
                "sa_threshold": 2.2,      # 加速度变化率阈值
                "energy_threshold": 3.5,  # 加速度能量阈值
                "dip_threshold": 0.3,     # 方向变化阈值
                "window_size": 150,       # 分析窗口大小(150个样本=3秒@50Hz)
                "sampling_rate": 50,       # 采样率(Hz)
            }

        self.config = config
        self.ra_threshold = config.get("ra_threshold", 1.85)
        self.sa_threshold = config.get("sa_threshold", 2.2)
        self.energy_threshold = config.get("energy_threshold", 3.5)
        self.dip_threshold = config.get("dip_threshold", 0.3)
        self.window_size = config.get("window_size", 150)
        self.sampling_rate = config.get("sampling_rate", 50)

        # 数据缓冲 - 存储(ax, ay, az)元组
        self.acc_buffer = deque(maxlen=self.window_size)

        # 跌倒检测状态
        self.fall_count = 0          # 连续跌倒检测次数
        self.normal_count = 0        # 连续正常次数
        self.current_state = "normal"  # normal, suspected, confirmed

    def check(self, acc_x, acc_y, acc_z):
        """
        检查是否跌倒

        参数:
            acc_x, acc_y, acc_z: 三轴加速度 (g)

        返回:
            (is_fall, details) - 是否跌倒及详情
        """
        # 添加新样本到缓冲区
        self.acc_buffer.append((acc_x, acc_y, acc_z))

        # 需要足够数据才能进行检测
        if len(self.acc_buffer) < self.window_size:
            return False, {
                "state": "collecting",
                "samples": len(self.acc_buffer),
                "required": self.window_size
            }

        # 论文公式(1): 计算合加速度 RA
        ra = self._calculate_ra()

        # 论文公式(2)-(3): 计算加速度变化率 SA
        sa = self._calculate_sa()

        # 论文公式(3): 计算加速度能量 E
        energy = self._calculate_energy()

        # 论文公式(4): 计算方向变化 D (dip)
        dip = self._calculate_dip()

        # 论文跌倒判定条件: RA > 1.85 && SA > 2.2 && E > 3.5 && Dip > 0.3
        is_suspected = (ra > self.ra_threshold and
                        sa > self.sa_threshold and
                        energy > self.energy_threshold and
                        dip > self.dip_threshold)

        # 状态机处理 - 连续3次检测到跌倒才确认
        if is_suspected:
            self.fall_count += 1
            self.normal_count = 0

            if self.fall_count >= 3:
                self.current_state = "confirmed"
            else:
                self.current_state = "suspected"
        else:
            self.normal_count += 1
            self.fall_count = 0

            if self.normal_count >= 10:
                self.current_state = "normal"
            elif self.fall_count > 0:
                self.current_state = "suspected"
            else:
                self.current_state = "normal"

        is_fall = (self.current_state == "confirmed")

        details = {
            "state": self.current_state,
            "ra": ra,
            "sa": sa,
            "energy": energy,
            "dip": dip,
            "ra_threshold": self.ra_threshold,
            "sa_threshold": self.sa_threshold,
            "energy_threshold": self.energy_threshold,
            "dip_threshold": self.dip_threshold,
            "fall_count": self.fall_count,
            "normal_count": self.normal_count,
            "buffer_size": len(self.acc_buffer),
        }

        return is_fall, details

    def _calculate_ra(self):
        """
        论文公式(1): 计算合加速度
        RA = √(ax² + ay² + az²)

        取窗口最后一个点的合加速度作为RA值
        """
        if not self.acc_buffer:
            return 1.0

        ax, ay, az = self.acc_buffer[-1]
        return math.sqrt(ax**2 + ay**2 + az**2)

    def _calculate_sa(self):
        """
        论文公式(2)-(3): 计算加速度变化率
        SA = (1/t) × [∫Ax(t)dt + ∫Ay(t)dt + ∫Az(t)dt]

        其中∫Ax(t)dt是Ax对时间的积分
        使用梯形积分法近似计算
        """
        n = len(self.acc_buffer)
        if n < 2:
            return 0.0

        # 窗口时长(秒)
        t = n / self.sampling_rate
        if t <= 0:
            return 0.0

        # 对三轴分别积分后求和
        # ∫Ax(t)dt ≈ Σ[(Ax[i] + Ax[i-1])/2 × Δt]
        integral_sum = 0.0
        dt = 1.0 / self.sampling_rate  # 采样间隔

        for i in range(1, n):
            ax_prev, ay_prev, az_prev = self.acc_buffer[i-1]
            ax_curr, ay_curr, az_curr = self.acc_buffer[i]

            # 梯形积分: (f(xi) + f(xi-1))/2 * dx
            integral_sum += (abs(ax_curr) + abs(ax_prev)) / 2 * dt
            integral_sum += (abs(ay_curr) + abs(ay_prev)) / 2 * dt
            integral_sum += (abs(az_curr) + abs(az_prev)) / 2 * dt

        # SA = (1/t) × integral
        sa = (1.0 / t) * integral_sum

        return sa

    def _calculate_energy(self):
        """
        论文公式(3): 计算加速度能量
        E = ∫s(t)²dt = ∫[Ax(t)² + Ay(t)² + Az(t)²]dt

        使用梯形积分法近似计算
        """
        n = len(self.acc_buffer)
        if n < 2:
            return 0.0

        dt = 1.0 / self.sampling_rate

        # 使用梯形积分法计算 ∫s(t)²dt
        energy = 0.0
        for i in range(1, n):
            ax_prev, ay_prev, az_prev = self.acc_buffer[i-1]
            ax_curr, ay_curr, az_curr = self.acc_buffer[i]

            # s(t)² = Ax² + Ay² + Az²
            s_prev_sq = ax_prev**2 + ay_prev**2 + az_prev**2
            s_curr_sq = ax_curr**2 + ay_curr**2 + az_curr**2

            # 梯形积分
            energy += (s_prev_sq + s_curr_sq) / 2 * dt

        return energy

    def _calculate_dip(self):
        """
        论文公式(4): 计算方向变化
        D = √[Σ(xi - yi)²]

        其中:
        - x为当前帧的三轴加速度
        - y为上一帧的三轴加速度
        - i = 1, 2, 3 表示x, y, z三个轴

        计算窗口内所有相邻帧方向变化的累积
        """
        n = len(self.acc_buffer)
        if n < 2:
            return 0.0

        # 计算所有相邻帧的方向变化并求和
        dip_sum = 0.0
        for i in range(1, n):
            ax_prev, ay_prev, az_prev = self.acc_buffer[i-1]
            ax_curr, ay_curr, az_curr = self.acc_buffer[i]

            # 论文公式(4): D = √[(x1-y1)² + (x2-y2)² + (x3-y3)²]
            # 其中x是当前帧, y是上一帧
            diff_x = ax_curr - ax_prev
            diff_y = ay_curr - ay_prev
            diff_z = az_curr - az_prev

            # Σ(xi - yi)²
            dip_sum += diff_x**2 + diff_y**2 + diff_z**2

        # 开平方
        dip = math.sqrt(dip_sum)

        # 论文公式(5): D = d_min + (d_max - d_min) × 20%
        # 这里d_min取0, d_max取dip的最大值, 简化为直接使用dip值
        # 根据论文实验, 方向变化阈值设为0.3
        dip_normalized = dip

        return dip_normalized

    def reset(self):
        """重置检测器"""
        self.acc_buffer.clear()
        self.fall_count = 0
        self.normal_count = 0
        self.current_state = "normal"

    def get_state(self):
        """获取当前状态"""
        return self.current_state
