# -*- coding: UTF-8 -*-
"""
步数检测模块

参考文献:
蒋博, 付乐乐. 基于腰部MEMS加速度计的多阈值步数检测算法[J]. 传感器与微系统, 2018.

核心算法:
1. 滑动窗口缓存: 大小为7
2. 三阶段检测:
   - Flag=1: 波峰阈值检测 (C[0-6] > T_max, 且中间第4个数最大)
   - Flag=2: 过零点检测 (C[3] < 0, C[2] > 0)
   - Flag=3: 波谷阈值检测 (C[0-6] < T_min, 且中间第4个数最小) → 步数+1, Flag重置为1
"""

import time
import math
from collections import deque


class StepDetector:
    """基于多阈值步数检测算法的步数检测器"""

    def __init__(self, config=None):
        if config is None:
            config = {
                "t_max": 0.12,           # 波峰阈值 (g)
                "t_min": -0.06,          # 波谷阈值 (g)
                "window_size": 7,        # 滑动窗口大小
            }

        self.config = config
        self.t_max = config.get("t_max", 0.12)
        self.t_min = config.get("t_min", -0.06)
        self.window_size = config.get("window_size", 7)

        # 核心状态
        self.step_count = 0
        self.flag = 1  # 1=波峰检测, 2=过零点检测, 3=波谷检测

        # 滑动窗口缓存
        self.buffer = deque(maxlen=self.window_size)

        # 最新的检测记录
        self.latest_record = None

        # 统计缓冲区
        self.stats_buffer = deque(maxlen=50)
        self.sample_count = 0

        # 导入重力去除器
        from .gravity_remover import GravityRemover
        self.gravity_remover = GravityRemover()

    def add_sample(self, acc_x, acc_y, acc_z, gyro_x=None, gyro_y=None, gyro_z=None, timestamp=None):
        """
        添加样本进行步数检测

        参数:
            acc_x, acc_y, acc_z: 三轴加速度(g)
            gyro_x, gyro_y, gyro_z: 三轴角速度(°/s)，可选
            timestamp: 可选时间戳

        返回:
            (是否检测到步数, 记录字典)
        """
        if timestamp is None:
            timestamp = time.time()

        # 计算原始合加速度
        acc_magnitude = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

        # 使用重力去除器处理（即使没有陀螺仪也进行重力去除）
        linear_acc = self.gravity_remover.add_sample(
            acc_x, acc_y, acc_z,
            gyro_x, gyro_y, gyro_z,
            timestamp
        )

        linear_x, linear_y, linear_z = linear_acc

        # 使用Y轴（垂直方向）投影作为合加速度
        # Y轴负方向向上，垂直方向加速度可有正负
        acc_deviation = linear_y

        # 更新统计缓冲区
        self.stats_buffer.append(acc_deviation)
        self.sample_count += 1

        # 添加到窗口
        self.buffer.append({
            "timestamp": timestamp,
            "acc": acc_deviation,
            "acc_raw": acc_magnitude,
            "ax": acc_x,
            "ay": acc_y,
            "az": acc_z,
            "linear_x": linear_x,
            "linear_y": linear_y,
            "linear_z": linear_z
        })

        # 窗口未满，无法检测
        if len(self.buffer) < self.window_size:
            record = self._create_record(
                detected=False,
                ax=acc_x,
                ay=acc_y,
                az=acc_z,
                acc_mag=acc_magnitude,
                acc_deviation=acc_deviation,
                reason="window_not_ready"
            )
            self.latest_record = record
            return False, record

        # 执行三阶段检测
        detected, record = self._detect_by_three_stage(acc_deviation, acc_magnitude, acc_x, acc_y, acc_z)

        self.latest_record = record
        return detected, record

    def _detect_by_three_stage(self, acc_deviation, acc_raw, ax, ay, az):
        """
        三阶段检测算法

        步骤:
        1. Flag=1: 波峰阈值检测 (C[0-6] > T_max, 且C[3]最大)
        2. Flag=2: 过零点检测 (C[3] < 0, C[2] > 0)
        3. Flag=3: 波谷阈值检测 (C[0-6] < T_min, 且C[3]最小) → 步数+1, Flag重置为1
        """
        window_data = list(self.buffer)
        acc_values = [s["acc"] for s in window_data]

        # 获取窗口中间值
        mid_idx = self.window_size // 2
        mid_acc = acc_values[mid_idx]

        # 获取C[mid_idx-1]
        c2_acc = acc_values[mid_idx - 1]

        # === Flag=1: 波峰阈值检测 ===
        if self.flag == 1:
            all_above_threshold = all(a > self.t_max for a in acc_values)
            # 波峰条件：中间点比其余6个数都大
            mid_is_peak = all(mid_acc > a for i, a in enumerate(acc_values) if i != mid_idx)

            if all_above_threshold and mid_is_peak:
                self.flag = 2
                return False, self._create_record(
                    detected=False,
                    ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                    acc_deviation=acc_deviation,
                    method="peak_detected",
                    reason="flag_1_to_2"
                )
            # 波峰条件不满足，flag保持为1
            return False, self._create_record(
                detected=False,
                ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                acc_deviation=acc_deviation,
                reason=f"flag_1_peak_not_found(all_above={all_above_threshold}, mid_peak={mid_is_peak})"
            )

        # === Flag=2: 过零点检测 ===
        elif self.flag == 2:
            if c2_acc > 0 and mid_acc < 0:
                self.flag = 3
                return False, self._create_record(
                    detected=False,
                    ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                    acc_deviation=acc_deviation,
                    method="zero_crossing",
                    reason="flag_2_to_3"
                )
            # 过零点条件不满足，flag保持为2
            return False, self._create_record(
                detected=False,
                ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                acc_deviation=acc_deviation,
                reason=f"flag_2_zero_not_found(c2={c2_acc:.3f}, mid={mid_acc:.3f})"
            )

        # === Flag=3: 波谷阈值检测 ===
        elif self.flag == 3:
            all_below_threshold = all(a < self.t_min for a in acc_values)
            # 波谷条件：中间点小于其余6个数
            mid_is_valley = all(mid_acc < a for i, a in enumerate(acc_values) if i != mid_idx)

            if all_below_threshold and mid_is_valley:
                # 检测到有效步数
                self.step_count += 1
                self.flag = 1  # 重置到第一阶段

                # 找到波谷对应的时间戳
                valley_time = window_data[mid_idx]["timestamp"]

                return True, self._create_record(
                    detected=True,
                    ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                    acc_deviation=acc_deviation,
                    method="valley_confirmed",
                    threshold_upper=self.t_max,
                    threshold_lower=self.t_min,
                    peak_time=valley_time,
                    reason="step_detected"
                )
            # 波谷条件不满足，flag保持为3
            return False, self._create_record(
                detected=False,
                ax=ax, ay=ay, az=az, acc_mag=acc_raw,
                acc_deviation=acc_deviation,
                reason=f"flag_3_valley_not_found(below={all_below_threshold}, valley={mid_is_valley})"
            )

        # 阶段未完成
        return False, self._create_record(
            detected=False,
            ax=ax, ay=ay, az=az, acc_mag=acc_raw,
            acc_deviation=acc_deviation,
            reason=f"flag_{self.flag}_processing"
        )

    def _create_record(self, detected, ax, ay, az, acc_mag, acc_deviation=0, method="none",
                       threshold_upper=None, threshold_lower=None,
                       peak_time=None, reason="none"):
        """创建检测记录"""
        # 计算当前统计值
        mean_acc = 0
        std_acc = 0
        if len(self.stats_buffer) > 1:
            mean_acc = sum(self.stats_buffer) / len(self.stats_buffer)
            variance = sum((x - mean_acc) ** 2 for x in self.stats_buffer) / len(self.stats_buffer)
            std_acc = math.sqrt(variance)

        record = {
            "detected": detected,
            "ax": ax,
            "ay": ay,
            "az": az,
            "acc_magnitude": acc_mag,
            "acc_deviation": acc_deviation,
            "timestamp": time.time(),
            "method": method,
            "threshold_upper": threshold_upper if threshold_upper is not None else self.t_max,
            "threshold_lower": threshold_lower if threshold_lower is not None else self.t_min,
            "flag": self.flag,
            "reason": reason,
            "peak_time": peak_time,
            "step_count": self.step_count,
            "buffer_size": len(self.buffer),
            "mean_acc": mean_acc,
            "std_acc": std_acc
        }

        return record

    def get_step_count(self):
        """获取累计步数"""
        return self.step_count

    def get_latest_record(self):
        """获取最新记录"""
        return self.latest_record

    def get_current_stats(self):
        """获取当前状态统计"""
        # 计算滑动窗口内的统计值
        if len(self.stats_buffer) > 1:
            mean_acc = sum(self.stats_buffer) / len(self.stats_buffer)
            variance = sum((x - mean_acc) ** 2 for x in self.stats_buffer) / len(self.stats_buffer)
            std_acc = math.sqrt(variance)
        else:
            mean_acc = 0
            std_acc = 0

        return {
            "step_count": self.step_count,
            "flag": self.flag,
            "threshold_upper": self.t_max,
            "threshold_lower": self.t_min,
            "buffer_size": len(self.buffer),
            "mean_acc": mean_acc,
            "std_acc": std_acc,
            "sample_count": self.sample_count
        }

    def reset(self):
        """重置检测器"""
        self.step_count = 0
        self.flag = 1
        self.buffer.clear()
        self.latest_record = None
        self.stats_buffer.clear()
        self.sample_count = 0
        if self.gravity_remover:
            self.gravity_remover.reset()

    def set_count(self, count):
        """设置步数"""
        self.step_count = count