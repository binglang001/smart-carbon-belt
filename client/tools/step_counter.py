# -*- coding: UTF-8 -*-
"""
步数计算程序

使用主程序的StepDetector模块进行步数检测
输入：采集的数据文件（包含预处理后的线性加速度）
输出：检测到的步数和统计信息
"""

import csv
import os
import sys
import math
import argparse

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入配置文件
import config

# 导入日志模块
from utils.logger import get_logger
logger = get_logger('tools.step_counter')


def load_data(csv_file):
    """加载CSV数据"""
    data = []
    with open(csv_file, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'timestamp': float(row['timestamp']),
                'sample_idx': int(row['sample_idx']),
                # 原始加速度
                'acc_x': float(row['acc_x']),
                'acc_y': float(row['acc_y']),
                'acc_z': float(row['acc_z']),
                'acc_magnitude': float(row['acc_magnitude']),
                # 预处理后的线性加速度
                'linear_x': float(row['linear_x']),
                'linear_y': float(row['linear_y']),
                'linear_z': float(row['linear_z']),
            })
    return data


def run_step_detection(csv_file):
    """
    运行步数检测

    参数:
        csv_file: CSV数据文件路径
    """
    # 加载数据
    data = load_data(csv_file)
    if not data:
        logger.error(f"无法加载数据: {csv_file}")
        return

    logger.info("=" * 50)
    logger.info("步数检测 - 使用主程序模块")
    logger.info("=" * 50)
    logger.info(f"数据文件: {csv_file}")
    logger.info(f"数据点数: {len(data)}")

    # 计算采样率
    if len(data) > 1:
        duration = data[-1]['timestamp'] - data[0]['timestamp']
        sample_rate = len(data) / duration if duration > 0 else 0
        logger.info(f"采样率: {sample_rate:.1f} Hz")
        logger.info(f"时长: {duration:.1f} 秒")

    # 使用主程序的StepDetector
    from sensors.step_detector import StepDetector

    # 从配置文件读取参数
    step_config = config.STEP_CONFIG
    gravity_config = config.GRAVITY_REMOVER_CONFIG

    detector_config = {
        "t_max": step_config.get("t_max", 0.12),
        "t_min": step_config.get("t_min", -0.06),
        "window_size": step_config.get("window_size", 7),
    }
    detector = StepDetector(detector_config)

    # 设置重力去除参数
    detector.gravity_remover.set_parameters(
        alpha=gravity_config.get("filter_alpha", 0.3),
        window=gravity_config.get("filter_window", 5)
    )

    logger.info(f"重力去除参数: alpha={gravity_config.get('filter_alpha')}, window={gravity_config.get('filter_window')}")
    logger.info(f"检测参数: T_max={detector_config['t_max']}, T_min={detector_config['t_min']}, "
                f"窗口大小={detector_config['window_size']}")

    # 逐样本添加进行步数检测
    steps_detected = []
    flag_counts = {1: 0, 2: 0, 3: 0}
    flag1_failures = {}
    flag2_failures = {}
    flag3_failures = {}

    for i, sample in enumerate(data):
        # 使用原始加速度，让 StepDetector 内部进行重力去除处理
        acc_x = sample['acc_x']
        acc_y = sample['acc_y']
        acc_z = sample['acc_z']
        timestamp = sample['timestamp']

        step_detected, record = detector.add_sample(
            acc_x, acc_y, acc_z,
            timestamp=timestamp
        )

        # 统计flag状态
        flag_counts[detector.flag] = flag_counts.get(detector.flag, 0) + 1

        # 统计各阶段失败原因
        reason = record.get('reason', '')
        if 'flag_1' in reason:
            flag1_failures[reason] = flag1_failures.get(reason, 0) + 1
        elif 'flag_2' in reason:
            flag2_failures[reason] = flag2_failures.get(reason, 0) + 1
        elif 'flag_3' in reason:
            flag3_failures[reason] = flag3_failures.get(reason, 0) + 1

        if step_detected:
            steps_detected.append(timestamp)

    logger.info(f"Flag状态统计: {flag_counts}")
    if flag1_failures:
        total_f1 = sum(flag1_failures.values())
        logger.info(f"Flag1失败: {total_f1}次 - {flag1_failures}")
    if flag2_failures:
        total_f2 = sum(flag2_failures.values())
        logger.info(f"Flag2失败: {total_f2}次 - {flag2_failures}")
    if flag3_failures:
        total_f3 = sum(flag3_failures.values())
        logger.info(f"Flag3失败: {total_f3}次 - {flag3_failures}")

    # 输出结果
    step_count = detector.get_step_count()
    logger.info(f"[检测结果]")
    logger.info(f"检测到的步数: {step_count}")

    if len(steps_detected) > 1:
        intervals = [steps_detected[i+1] - steps_detected[i]
                    for i in range(len(steps_detected)-1)]
        avg_interval = sum(intervals) / len(intervals)
        avg_step_freq = 1.0 / avg_interval if avg_interval > 0 else 0
        logger.info(f"平均步频: {avg_step_freq:.1f} 步/分钟")
        logger.info(f"平均步间隔: {avg_interval*1000:.0f} ms")

    return step_count


def find_latest_data_file():
    """查找最新的数据文件"""
    search_dirs = ['data/gravity', 'data']
    csv_files = []

    for data_dir in search_dirs:
        if os.path.exists(data_dir):
            for f in os.listdir(data_dir):
                if f.endswith('.csv'):
                    csv_files.append(os.path.join(data_dir, f))

    if not csv_files:
        return None

    csv_files.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    return csv_files[0]


def main():
    parser = argparse.ArgumentParser(description='步数检测 - 使用主程序模块')
    parser.add_argument('file', nargs='?', help='CSV数据文件路径（默认自动查找最新）')

    args = parser.parse_args()

    # 自动查找最新文件
    input_file = args.file
    if not input_file:
        input_file = find_latest_data_file()
        if not input_file:
            logger.error("未找到数据文件，请先运行 data_collector.py 采集数据")
            return

    if not os.path.exists(input_file):
        logger.error(f"文件不存在: {input_file}")
        return

    logger.info(f"分析文件: {input_file}")
    run_step_detection(input_file)


if __name__ == '__main__':
    main()
