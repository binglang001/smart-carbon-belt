# -*- coding: UTF-8 -*-
"""
数据分析程序

分析采集到的步态数据，输出论文要求的所有参数和分析结果
重点分析预处理后的线性加速度数据
"""

import csv
import os
import sys
import math
from collections import deque
import argparse

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入日志模块
from utils.logger import get_logger
logger = get_logger('tools.data_analyzer')


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


def calculate_basic_stats(values):
    """计算基本统计量"""
    if not values:
        return {}

    n = len(values)
    mean = sum(values) / n
    variance = sum((x - mean) ** 2 for x in values) / n
    std = math.sqrt(variance)
    max_val = max(values)
    min_val = min(values)

    return {
        'count': n,
        'mean': mean,
        'variance': variance,
        'std': std,
        'max': max_val,
        'min': min_val,
        'range': max_val - min_val,
    }


def analyze_raw_acceleration(data):
    """分析原始加速度数据"""
    logger.info("\n" + "=" * 60)
    logger.info("[原始加速度统计] 论文1: A = √(Ax² + Ay² + Az²)")
    logger.info("=" * 60)

    acc_mag = [d['acc_magnitude'] for d in data]
    stats = calculate_basic_stats(acc_mag)

    logger.info(f"  样本数: {stats['count']}")
    logger.info(f"  平均值: {stats['mean']:.4f} g")
    logger.info(f"  标准差: {stats['std']:.4f} g")
    logger.info(f"  最大值: {stats['max']:.4f} g")
    logger.info(f"  最小值: {stats['min']:.4f} g")
    logger.info(f"  范围: {stats['range']:.4f} g")

    return stats


def analyze_linear_acceleration(data):
    """
    分析预处理后的线性加速度数据

    论文1: 利用去除重力加速度g的xyz三轴合加速度的模值作为初始检测数据
    论文2: 双向角度法去除重力
    """
    logger.info("\n" + "=" * 60)
    logger.info("[预处理后线性加速度统计] 论文1+论文2: 去除重力后的合加速度")
    logger.info("=" * 60)

    linear_y = [d['linear_y'] for d in data]
    stats = calculate_basic_stats(linear_y)

    logger.info(f"  样本数: {stats['count']}")
    logger.info(f"  平均值: {stats['mean']:.4f} g")
    logger.info(f"  标准差: {stats['std']:.4f} g")
    logger.info(f"  最大值: {stats['max']:.4f} g")
    logger.info(f"  最小值: {stats['min']:.4f} g")
    logger.info(f"  范围: {stats['range']:.4f} g")

    # 线性加速度三分量统计
    linear_x = [d['linear_x'] for d in data]
    linear_y = [d['linear_y'] for d in data]
    linear_z = [d['linear_z'] for d in data]

    logger.info(f"\n  [三分量统计]")
    x_stats = calculate_basic_stats(linear_x)
    y_stats = calculate_basic_stats(linear_y)
    z_stats = calculate_basic_stats(linear_z)

    logger.info(f"    X: mean={x_stats['mean']:.4f}, std={x_stats['std']:.4f}, range=[{x_stats['min']:.4f}, {x_stats['max']:.4f}]")
    logger.info(f"    Y: mean={y_stats['mean']:.4f}, std={y_stats['std']:.4f}, range=[{y_stats['min']:.4f}, {y_stats['max']:.4f}]")
    logger.info(f"    Z: mean={z_stats['mean']:.4f}, std={z_stats['std']:.4f}, range=[{z_stats['min']:.4f}, {z_stats['max']:.4f}]")

    return stats


def analyze_zero_crossings(data):
    """
    分析过零点

    论文1 步骤3（过零点检测）:
    条件1: C[3] < 0
    条件2: C[2] > 0
    """
    logger.info("\n" + "=" * 60)
    logger.info("[过零点分析] 论文1: C[3] < 0 且 C[2] > 0")
    logger.info("=" * 60)

    linear_y = [d['linear_y'] for d in data]

    # 统计过零次数 (从正到负和从负到正)
    pos_to_neg = 0  # 从正到负
    neg_to_pos = 0  # 从负到正

    for i in range(1, len(linear_y)):
        prev = linear_y[i-1]
        curr = linear_y[i]

        if prev > 0 and curr <= 0:
            pos_to_neg += 1
        elif prev <= 0 and curr > 0:
            neg_to_pos += 1

    total_crossings = pos_to_neg + neg_to_pos

    logger.info(f"  总过零次数: {total_crossings}")
    logger.info(f"  从正到负: {pos_to_neg}")
    logger.info(f"  从负到正: {neg_to_pos}")
    logger.info(f"  (注意: 使用线性加速度，即去除重力后的值)")

    return {
        'total': total_crossings,
        'pos_to_neg': pos_to_neg,
        'neg_to_pos': neg_to_pos
    }


def analyze_peaks_and_valleys(data):
    """
    分析波峰和波谷

    论文1:
    - 波峰条件: C[0-6] > T_max 且 C[3]最大
    - 波谷条件: C[0-6] < T_min 且 C[3]最小
    """
    logger.info("\n" + "=" * 60)
    logger.info("[波峰波谷分析] 论文1: 窗口7个数据")
    logger.info("=" * 60)

    linear_y = [d['linear_y'] for d in data]
    window_size = 7

    # 简单波峰波谷（窗口内相对极值）
    peaks = []
    valleys = []

    half = window_size // 2

    for i in range(half, len(linear_y) - half):
        window = linear_y[i-half:i+half+1]
        mid_val = window[half]

        # 波峰
        if mid_val == max(window):
            peaks.append({'index': i, 'value': mid_val})

        # 波谷
        if mid_val == min(window):
            valleys.append({'index': i, 'value': mid_val})

    logger.info(f"  简单波峰数: {len(peaks)}")
    logger.info(f"  简单波谷数: {len(valleys)}")

    return {'peaks': peaks, 'valleys': valleys}


def analyze_three_stage_detection(data, real_step_count=None):
    """
    分析论文要求的三阶段阈值检测

    论文1 步骤2-4:
    - 波峰检测: C[0] > Tmax 且 ... 且 C[6] > Tmax, 且 C[3]最大
    - 过零检测: C[3] < 0 且 C[2] > 0
    - 波谷检测: C[0] < Tmin 且 ... 且 C[6] < Tmin, 且 C[3]最小

    注意: 论文没有给出T_max和T_min的具体数值，需要标定
    """
    logger.info("\n" + "=" * 60)
    logger.info("[三阶段阈值检测分析] 论文1: 需要标定T_max和T_min")
    logger.info("=" * 60)

    linear_y = [d['linear_y'] for d in data]
    window_size = 7
    mid_idx = 3

    # 计算合理的阈值范围
    mean_val = sum(linear_y) / len(linear_y)
    std_val = math.sqrt(sum((x - mean_val)**2 for x in linear_y) / len(linear_y))

    logger.info(f"\n  [基于统计的建议阈值范围]")
    logger.info(f"    均值: {mean_val:.4f} g")
    logger.info(f"    标准差: {std_val:.4f} g")
    logger.info(f"    建议T_max范围: {mean_val + std_val:.4f} ~ {mean_val + 2*std_val:.4f}")
    logger.info(f"    建议T_min范围: {mean_val - 2*std_val:.4f} ~ {mean_val - std_val:.4f}")

    # 测试不同阈值
    logger.info(f"\n  [不同阈值下的检测结果]")

    # T_max候选值
    t_max_candidates = [
        mean_val + 0.5 * std_val,
        mean_val + std_val,
        mean_val + 1.5 * std_val,
        mean_val + 2.0 * std_val,
    ]

    # T_min候选值
    t_min_candidates = [
        mean_val - 2.0 * std_val,
        mean_val - 1.5 * std_val,
        mean_val - std_val,
        mean_val - 0.5 * std_val,
    ]

    logger.info(f"\n  {'T_max':<10} {'T_min':<10} {'波峰候选':<12} {'过零条件':<12} {'波谷候选':<12}")
    logger.info("  " + "-" * 60)

    results = []

    for t_max in t_max_candidates:
        for t_min in t_min_candidates:
            if t_min >= t_max:
                continue

            # 统计三阶段候选数
            peak_candidates = 0
            zero_ok = 0
            valley_candidates = 0

            for i in range(window_size - 1, len(linear_y) - window_size + 1):
                window = linear_y[i-window_size+1:i+1]
                mid_val = window[mid_idx]
                c2_val = window[mid_idx - 1]

                # 波峰检测
                if all(v > t_max for v in window) and all(mid_val >= v for j, v in enumerate(window) if j != mid_idx):
                    peak_candidates += 1

                # 过零检测
                if c2_val > 0 and mid_val < 0:
                    zero_ok += 1

                # 波谷检测
                if all(v < t_min for v in window) and all(mid_val <= v for j, v in enumerate(window) if j != mid_idx):
                    valley_candidates += 1

            results.append({
                't_max': t_max,
                't_min': t_min,
                'peaks': peak_candidates,
                'zero': zero_ok,
                'valleys': valley_candidates
            })

            logger.info(f"  {t_max:<10.4f} {t_min:<10.4f} {peak_candidates:<12} {zero_ok:<12} {valley_candidates:<12}")

    return results


def analyze_data(csv_file, real_step_count=None):
    """完整分析数据"""
    logger.info("=" * 60)
    logger.info(f"步态数据分析")
    logger.info(f"文件: {csv_file}")
    logger.info("=" * 60)

    # 加载数据
    data = load_data(csv_file)
    logger.info(f"\n[基本信息]")
    logger.info(f"  样本数量: {len(data)}")

    if len(data) > 1:
        duration = data[-1]['timestamp'] - data[0]['timestamp']
        logger.info(f"  持续时间: {duration:.2f} 秒")
        logger.info(f"  实际采样率: {len(data)/duration:.1f} Hz")

    if real_step_count:
        logger.info(f"  实际步数: {real_step_count}")

    # 1. 原始加速度统计
    raw_stats = analyze_raw_acceleration(data)

    # 2. 预处理后线性加速度统计
    linear_stats = analyze_linear_acceleration(data)

    # 3. 过零点分析
    zero_stats = analyze_zero_crossings(data)

    # 4. 波峰波谷分析
    pv_stats = analyze_peaks_and_valleys(data)

    # 5. 三阶段阈值检测分析
    three_stage_results = analyze_three_stage_detection(data, real_step_count)

    # 总结
    logger.info("\n" + "=" * 60)
    logger.info("[分析总结]")
    logger.info("=" * 60)
    logger.info(f"  原始合加速度均值: {raw_stats['mean']:.4f} g (应接近1g)")
    logger.info(f"  线性加速度均值: {linear_stats['mean']:.4f} g (应接近0)")
    logger.info(f"  线性加速度标准差: {linear_stats['std']:.4f} g")
    logger.info(f"  过零次数: {zero_stats['total']}")

    return {
        'raw_stats': raw_stats,
        'linear_stats': linear_stats,
        'zero_stats': zero_stats,
    }


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
    parser = argparse.ArgumentParser(description='步态数据分析 - 论文算法')
    parser.add_argument('input', nargs='?', help='输入CSV文件（默认自动查找最新）')
    parser.add_argument('-r', '--real-steps', type=int, help='实际步数（用于比对）')

    args = parser.parse_args()

    # 自动查找最新文件
    input_file = args.input
    if not input_file:
        input_file = find_latest_data_file()
        if not input_file:
            logger.error("未找到数据文件，请先运行 data_collector.py 采集数据")
            return

    if not os.path.exists(input_file):
        logger.error(f"文件不存在: {input_file}")
        return

    logger.info(f"分析文件: {input_file}")
    analyze_data(input_file, args.real_steps)


if __name__ == '__main__':
    main()
