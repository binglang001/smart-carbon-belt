# -*- coding: UTF-8 -*-
"""
数据绘图程序

绘制采集数据的图表
使用预处理后的线性加速度数据
"""

import csv
import os
import sys
import math
import argparse
import platform

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入配置文件
import config

# 导入日志模块
from utils.logger import get_logger
logger = get_logger('tools.data_plotter')

# 导入重力去除器
from sensors.gravity_remover import GravityRemover

# 从配置文件读取默认参数
DEFAULT_WINDOW_SIZE = config.STEP_CONFIG.get("window_size", 7)

# 配置matplotlib中文字体
def setup_chinese_font():
    """配置中文字体支持"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # 根据操作系统选择字体
    if platform.system() == 'Windows':
        # Windows中文字体
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'Arial']
    elif platform.system() == 'Darwin':
        # macOS
        plt.rcParams['font.sans-serif'] = ['PingFang SC', 'Hiragino Sans GB', 'Arial Unicode MS']
    else:
        # Linux
        plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Droid Sans Fallback']

    # 解决负号显示问题
    plt.rcParams['axes.unicode_minus'] = False

    return plt


def load_data(csv_file):
    """加载CSV数据（包括原始加速度，用于重力去除）"""
    data = []
    with open(csv_file, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'timestamp': float(row['timestamp']),
                'sample_idx': int(row['sample_idx']),
                'acc_x': float(row['acc_x']),
                'acc_y': float(row['acc_y']),
                'acc_z': float(row['acc_z']),
                'acc_magnitude': float(row['acc_magnitude']),
            })
    return data


def preprocess_data(data):
    """使用重力去除器处理原始加速度数据"""
    # 从配置文件读取重力去除参数
    gravity_config = config.GRAVITY_REMOVER_CONFIG
    filter_alpha = gravity_config.get("filter_alpha", 0.3)
    filter_window = gravity_config.get("filter_window", 5)

    remover = GravityRemover({
        "filter_alpha": filter_alpha,
        "filter_window": filter_window
    })

    processed = []
    for sample in data:
        linear_x, linear_y, linear_z = remover.add_sample(
            sample['acc_x'], sample['acc_y'], sample['acc_z'],
            timestamp=sample['timestamp']
        )
        processed.append({
            'timestamp': sample['timestamp'],
            'sample_idx': sample['sample_idx'],
            'acc_magnitude': sample['acc_magnitude'],
            'linear_y': linear_y,
        })

    logger.info(f"重力去除参数: alpha={filter_alpha}, window={filter_window}")
    return processed


def find_three_stage_features(data, t_max, t_min, window_size=None):
    """
    三阶段特征检测 - 完全基于论文算法

    阶段1 (Flag=1): 波峰阈值检测 - 所有窗口点 > T_max 且 中间点最大
    阶段2 (Flag=2): 过零点检测 - C[2] > 0 且 C[3] < 0
    阶段3 (Flag=3): 波谷阈值检测 - 所有窗口点 < T_min 且 中间点最小
    """
    if window_size is None:
        window_size = DEFAULT_WINDOW_SIZE
    mid_idx = window_size // 2

    linear_mag = [d['linear_y'] for d in data]
    timestamps = [d['timestamp'] for d in data]

    peaks = []       # Flag=1 候选点
    zero_crossings = []  # Flag=2 候选点
    valleys = []     # Flag=3 候选点

    # 用于记录某个波峰索引后面紧跟的过零点
    peak_to_zero = {}  # {波峰索引: [过零点索引列表]}

    for i in range(window_size - 1, len(linear_mag) - window_size + 1):
        window = linear_mag[i-mid_idx:i+mid_idx+1]
        mid_val = window[mid_idx]
        c2_val = window[mid_idx - 1]

        # === Flag=1: 波峰检测 ===
        all_above = all(v > t_max for v in window)
        mid_is_max = all(mid_val >= v for j, v in enumerate(window) if j != mid_idx)
        if all_above and mid_is_max:
            peaks.append(i)

        # === Flag=2: 过零点检测 (要求在波峰之后) ===
        if c2_val > 0 and mid_val < 0:
            # 找最近的波峰
            for p in reversed(peaks):
                if p < i:
                    if p not in peak_to_zero:
                        peak_to_zero[p] = []
                    peak_to_zero[p].append(i)
                    break
            zero_crossings.append(i)

        # === Flag=3: 波谷检测 (要求在过零点之后) ===
        all_below = all(v < t_min for v in window)
        mid_is_min = all(mid_val <= v for j, v in enumerate(window) if j != mid_idx)
        if all_below and mid_is_min:
            valleys.append(i)

    return {
        'peaks': peaks,
        'zero_crossings': zero_crossings,
        'valleys': valleys,
        'peak_to_zero': peak_to_zero
    }


def plot_with_matplotlib(data, output_dir=None, t_max=1.0, t_min=-0.5):
    """使用matplotlib绘图"""
    try:
        plt = setup_chinese_font()
    except ImportError:
        logger.error("需要安装 matplotlib")
        logger.info("请运行: pip install matplotlib")
        return

    # 提取数据
    timestamps = [d['timestamp'] - data[0]['timestamp'] for d in data]
    acc_mag = [d['acc_magnitude'] for d in data]
    linear_mag = [d['linear_y'] for d in data]

    # 使用三阶段检测
    features = find_three_stage_features(data, t_max, t_min)
    peaks = features['peaks']
    valleys = features['valleys']
    zero_crossings = features['zero_crossings']

    # 创建输出目录
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(output_dir) if output_dir else '.', "plots")
    os.makedirs(output_dir, exist_ok=True)

    dpi = 300

    # 图1: 原始加速度
    fig1, ax1 = plt.subplots(figsize=(14, 5))
    ax1.plot(timestamps, acc_mag, 'b-', linewidth=0.8, label='原始合加速度')
    ax1.axhline(y=1.0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5, label='1g')
    ax1.set_xlabel('时间 (秒)')
    ax1.set_ylabel('加速度 (g)')
    ax1.set_title('原始加速度合量')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    output_file1 = os.path.join(output_dir, '01_raw_acceleration.png')
    plt.savefig(output_file1, dpi=dpi)
    logger.info(f"图表已保存: {output_file1}")
    plt.close()

    # 图2: 预处理后线性加速度（带特征点标记）
    fig2, ax2 = plt.subplots(figsize=(14, 5))
    ax2.plot(timestamps, linear_mag, 'b-', linewidth=0.8, label='线性加速度')

    # 阈值线
    ax2.axhline(y=t_max, color='r', linestyle='--', linewidth=1, label=f'T_max={t_max}')
    ax2.axhline(y=t_min, color='g', linestyle='--', linewidth=1, label=f'T_min={t_min}')
    ax2.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5, label='零线')

    # 标记波峰
    if peaks:
        peak_times = [timestamps[p] for p in peaks]
        peak_vals = [linear_mag[p] for p in peaks]
        ax2.scatter(peak_times, peak_vals, c='red', marker='^', s=30, zorder=5, label=f'波峰({len(peaks)})')

    # 标记波谷
    if valleys:
        valley_times = [timestamps[v] for v in valleys]
        valley_vals = [linear_mag[v] for v in valleys]
        ax2.scatter(valley_times, valley_vals, c='green', marker='v', s=30, zorder=5, label=f'波谷({len(valleys)})')

    # 标记过零点
    if zero_crossings:
        zc_times = [timestamps[z] for z in zero_crossings]
        zc_vals = [linear_mag[z] for z in zero_crossings]
        ax2.scatter(zc_times, zc_vals, c='orange', marker='o', s=15, zorder=4, alpha=0.5, label=f'过零({len(zero_crossings)})')

    ax2.set_xlabel('时间 (秒)')
    ax2.set_ylabel('加速度 (g)')
    ax2.set_title('预处理后线性加速度 - 带特征点标记')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    output_file2 = os.path.join(output_dir, '02_linear_with_peaks.png')
    plt.savefig(output_file2, dpi=dpi)
    logger.info(f"图表已保存: {output_file2}")
    plt.close()

    logger.info(f"所有图表已保存到目录: {output_dir}")


def plot_with_text(data, t_max=1.0, t_min=-0.5):
    """纯文本绘图 - 使用三阶段检测"""
    features = find_three_stage_features(data, t_max, t_min)
    peaks = features['peaks']
    valleys = features['valleys']
    zero_crossings = features['zero_crossings']
    peak_to_zero = features['peak_to_zero']

    # 计算数据统计
    acc_mags = [d['acc_magnitude'] for d in data]
    linear_ys = [d['linear_y'] for d in data]

    import math
    avg_acc = sum(acc_mags) / len(acc_mags) if acc_mags else 0
    max_acc = max(acc_mags) if acc_mags else 0
    min_acc = min(acc_mags) if acc_mags else 0

    avg_linear = sum(linear_ys) / len(linear_ys) if linear_ys else 0
    max_linear = max(linear_ys) if linear_ys else 0
    min_linear = min(linear_ys) if linear_ys else 0

    # 统计完整三阶段的步数
    valid_steps = 0
    for valley_idx in valleys:
        # 找这个波谷之前的过零点
        for zc_idx in reversed(zero_crossings):
            if zc_idx < valley_idx:
                # 找这个过零点之前的波峰
                for peak_idx in reversed(peaks):
                    if peak_idx < zc_idx:
                        valid_steps += 1
                        break
                break

    logger.info("\n" + "=" * 50)
    logger.info("三阶段检测结果（基于论文算法）")
    logger.info("=" * 50)
    logger.info(f"样本数: {len(data)}")
    logger.info(f"原始加速度 - 均值: {avg_acc:.4f}g, 范围: [{min_acc:.4f}, {max_acc:.4f}]")
    logger.info(f"线性加速度Y - 均值: {avg_linear:.4f}g, 范围: [{min_linear:.4f}, {max_linear:.4f}]")
    logger.info(f"参数: T_max={t_max}, T_min={t_min}")
    logger.info("-" * 50)
    logger.info(f"阶段1 - 波峰候选数 (C[0-4] > T_max 且 中间点最大): {len(peaks)}")
    logger.info(f"阶段2 - 过零点数 (C[1] > 0 且 C[2] < 0): {len(zero_crossings)}")
    logger.info(f"阶段3 - 波谷候选数 (C[0-4] < T_min 且 中间点最小): {len(valleys)}")
    logger.info("-" * 50)
    logger.info(f"完整三阶段检测到的步数: {valid_steps}")


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
    # 从配置文件读取默认参数
    DEFAULT_T_MAX = config.STEP_CONFIG.get("t_max", 0.12)
    DEFAULT_T_MIN = config.STEP_CONFIG.get("t_min", -0.06)

    parser = argparse.ArgumentParser(description='步数检测分析 - 绘图工具')
    parser.add_argument('input', nargs='?', help='输入CSV文件（默认自动查找最新）')
    parser.add_argument('-o', '--output', help='输出图片目录（默认自动生成）')
    parser.add_argument('--t-max', type=float, default=DEFAULT_T_MAX, help=f'波峰阈值 (默认{DEFAULT_T_MAX})')
    parser.add_argument('--t-min', type=float, default=DEFAULT_T_MIN, help=f'波谷阈值 (默认{DEFAULT_T_MIN})')
    parser.add_argument('--text', action='store_true', help='纯文本模式')

    args = parser.parse_args()

    # 自动查找最新文件
    input_file = args.input
    if not input_file:
        input_file = find_latest_data_file()
        if not input_file:
            logger.error("未找到数据文件，请先运行 data_collector.py 采集数据")
            sys.exit(1)

    if not os.path.exists(input_file):
        logger.error(f"文件不存在: {input_file}")
        return

    # 自动生成输出目录
    if not args.output:
        base_name = os.path.splitext(os.path.basename(input_file))[0]
        args.output = os.path.join(os.path.dirname(input_file), "plots", base_name)
        logger.info(f"自动保存图表目录: {args.output}")

    raw_data = load_data(input_file)

    # 使用重力去除器处理原始加速度数据
    data = preprocess_data(raw_data)

    if args.text:
        plot_with_text(data, args.t_max, args.t_min)
    else:
        try:
            plot_with_matplotlib(data, args.output, args.t_max, args.t_min)
        except ImportError:
            logger.warning("matplotlib 不可用，使用文本模式")
            plot_with_text(data, args.t_max, args.t_min)


if __name__ == '__main__':
    main()
