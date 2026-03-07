# -*- coding: UTF-8 -*-
"""
重力去除分析器

用于分析不同参数下的重力去除效果
支持自定义滤波系数和滑动窗口大小
"""

import csv
import os
import sys
import math
import argparse
import platform

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入日志模块
from utils.logger import get_logger
logger = get_logger('tools.gravity_analyzer')


def setup_chinese_font():
    """配置中文字体支持"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if platform.system() == 'Windows':
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'Arial']
    elif platform.system() == 'Darwin':
        plt.rcParams['font.sans-serif'] = ['PingFang SC', 'Hiragino Sans GB', 'Arial Unicode MS']
    else:
        plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Droid Sans Fallback']

    plt.rcParams['axes.unicode_minus'] = False
    return plt


class GravityAnalyzer:
    """重力去除分析器"""

    def __init__(self, csv_file, step_t_max=0.15, step_t_min=-0.05):
        self.csv_file = csv_file
        self.raw_data = []
        self.processed_data = []
        # 步数检测阈值
        self.step_t_max = step_t_max
        self.step_t_min = step_t_min

    def load_data(self):
        """加载CSV数据"""
        if not os.path.exists(self.csv_file):
            logger.error(f"文件不存在: {self.csv_file}")
            return False

        with open(self.csv_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.raw_data.append({
                    'timestamp': float(row['timestamp']),
                    'sample_idx': int(row['sample_idx']),
                    'acc_x': float(row['acc_x']),
                    'acc_y': float(row['acc_y']),
                    'acc_z': float(row['acc_z']),
                    'acc_magnitude': float(row['acc_magnitude']),
                    'linear_x': float(row['linear_x']),
                    'linear_y': float(row['linear_y']),
                    'linear_z': float(row['linear_z']),
                    'linear_magnitude': float(row['linear_magnitude']),
                })

        logger.info(f"加载了 {len(self.raw_data)} 条记录")
        return True

    def process_with_params(self, alpha, window_size):
        """
        使用指定参数处理数据

        参数:
            alpha: 滤波系数 (0-1)
            window_size: 滑动平均窗口大小
        """
        if not self.raw_data:
            logger.error("无原始数据，请先加载数据")
            return []

        self.processed_data = []

        # 重力加速度初始值
        gravity_x = 0.0
        gravity_y = 0.0
        gravity_z = 0.0
        is_first = True

        # 滑动平均缓冲区
        buffer_x = []
        buffer_y = []
        buffer_z = []

        for i, data in enumerate(self.raw_data):
            acc_x = data['acc_x']
            acc_y = data['acc_y']
            acc_z = data['acc_z']

            if is_first:
                gravity_x = 0.0
                gravity_y = 0.0
                gravity_z = 0.0
                linear_x = acc_x
                linear_y = acc_y
                linear_z = acc_z
                is_first = False
            else:
                # 低通滤波
                gravity_x = alpha * acc_x + (1 - alpha) * gravity_x
                gravity_y = alpha * acc_y + (1 - alpha) * gravity_y
                gravity_z = alpha * acc_z + (1 - alpha) * gravity_z

                # 线性加速度
                linear_x = acc_x - gravity_x
                linear_y = acc_y - gravity_y
                linear_z = acc_z - gravity_z

            # 滑动平均滤波
            buffer_x.append(linear_x)
            buffer_y.append(linear_y)
            buffer_z.append(linear_z)

            if len(buffer_x) > window_size:
                buffer_x.pop(0)
                buffer_y.pop(0)
                buffer_z.pop(0)

            if len(buffer_x) >= window_size:
                linear_x = sum(buffer_x) / len(buffer_x)
                linear_y = sum(buffer_y) / len(buffer_y)
                linear_z = sum(buffer_z) / len(buffer_z)

            linear_mag = math.sqrt(linear_x**2 + linear_y**2 + linear_z**2)

            self.processed_data.append({
                'timestamp': data['timestamp'],
                'sample_idx': data['sample_idx'],
                'gravity_x': gravity_x,
                'gravity_y': gravity_y,
                'gravity_z': gravity_z,
                'linear_x': linear_x,
                'linear_y': linear_y,
                'linear_z': linear_z,
                'linear_magnitude': linear_mag,
            })

        logger.info(f"参数: alpha={alpha}, window={window_size}, 处理了 {len(self.processed_data)} 条数据")
        return self.processed_data

    def analyze_statistics(self, data):
        """分析统计数据"""
        if not data:
            logger.info("无数据")
            return

        linear_mags = [d['linear_magnitude'] for d in data]
        linear_ys = [d['linear_y'] for d in data]

        avg_mag = sum(linear_mags) / len(linear_mags) if linear_mags else 0
        max_mag = max(linear_mags) if linear_mags else 0
        min_mag = min(linear_mags) if linear_mags else 0

        avg_y = sum(linear_ys) / len(linear_ys) if linear_ys else 0
        max_y = max(linear_ys) if linear_ys else 0
        min_y = min(linear_ys) if linear_ys else 0

        logger.info("\n" + "=" * 50)
        logger.info("重力去除统计")
        logger.info("=" * 50)
        logger.info(f"样本数: {len(data)}")
        logger.info(f"线性加速度幅值 - 均值: {avg_mag:.4f}g, 范围: [{min_mag:.4f}, {max_mag:.4f}]")
        logger.info(f"线性加速度Y轴 - 均值: {avg_y:.4f}g, 范围: [{min_y:.4f}, {max_y:.4f}]")

    def plot_comparison(self, alpha, window_size, output_dir=None, t_max=None, t_min=None):
        """绘制对比图"""
        try:
            plt = setup_chinese_font()
        except ImportError:
            logger.error("需要安装 matplotlib")
            return

        if not self.raw_data:
            logger.error("无原始数据")
            return

        # 使用传入的参数或类属性
        if t_max is None:
            t_max = self.step_t_max
        if t_min is None:
            t_min = self.step_t_min

        # 处理数据
        self.process_with_params(alpha, window_size)

        # 提取数据
        timestamps = [d['timestamp'] - self.raw_data[0]['timestamp'] for d in self.raw_data]
        raw_mag = [d['acc_magnitude'] for d in self.raw_data]
        linear_mag = [d['linear_magnitude'] for d in self.processed_data]
        linear_y = [d['linear_y'] for d in self.processed_data]

        # 创建输出目录
        if output_dir is None:
            base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
            output_dir = os.path.join(os.path.dirname(self.csv_file), "plots", base_name)
        os.makedirs(output_dir, exist_ok=True)

        # DPI设置
        dpi = 300

        # 图1: 原始加速度幅值
        fig1, ax1 = plt.subplots(figsize=(14, 5))
        ax1.plot(timestamps, raw_mag, 'b-', linewidth=0.8, label='原始加速度幅值')
        ax1.axhline(y=1.0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5, label='1g')
        ax1.set_xlabel('时间 (秒)')
        ax1.set_ylabel('加速度 (g)')
        ax1.set_title('原始加速度幅值')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file1 = os.path.join(output_dir, '01_raw_acceleration.png')
        plt.savefig(output_file1, dpi=dpi)
        logger.info(f"图表已保存: {output_file1}")
        plt.close()

        # 图2: 线性加速度Y轴分量（步数检测用这个）
        fig2, ax2 = plt.subplots(figsize=(14, 5))
        ax2.plot(timestamps, linear_y, 'g-', linewidth=0.8, label=f'线性加速度Y轴 (α={alpha}, window={window_size})')
        ax2.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5, label='零线')
        ax2.axhline(y=t_max, color='r', linestyle='--', linewidth=0.5, alpha=0.3, label=f'T_max={t_max}')
        ax2.axhline(y=t_min, color='r', linestyle='--', linewidth=0.5, alpha=0.3, label=f'T_min={t_min}')
        ax2.set_xlabel('时间 (秒)')
        ax2.set_ylabel('加速度 (g)')
        ax2.set_title('线性加速度Y轴分量（步数检测用此通道）')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file2 = os.path.join(output_dir, '02_linear_y_axis.png')
        plt.savefig(output_file2, dpi=dpi)
        logger.info(f"图表已保存: {output_file2}")
        plt.close()

        # 图3: 线性加速度幅值
        fig3, ax3 = plt.subplots(figsize=(14, 5))
        ax3.plot(timestamps, linear_mag, 'r-', linewidth=0.8, label=f'线性加速度幅值')
        ax3.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5, label='零线')
        ax3.set_xlabel('时间 (秒)')
        ax3.set_ylabel('加速度 (g)')
        ax3.set_title('重力去除后的线性加速度幅值')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file3 = os.path.join(output_dir, '03_linear_magnitude.png')
        plt.savefig(output_file3, dpi=dpi)
        logger.info(f"图表已保存: {output_file3}")
        plt.close()

        logger.info(f"所有图表已保存到目录: {output_dir}")

        # 输出统计
        self.analyze_statistics(self.processed_data)

    def plot_original(self, output_dir=None):
        """绘制原始数据图"""
        try:
            plt = setup_chinese_font()
        except ImportError:
            logger.error("需要安装 matplotlib")
            return

        if not self.raw_data:
            logger.error("无原始数据")
            return

        # 提取数据
        timestamps = [d['timestamp'] - self.raw_data[0]['timestamp'] for d in self.raw_data]
        raw_x = [d['acc_x'] for d in self.raw_data]
        raw_y = [d['acc_y'] for d in self.raw_data]
        raw_z = [d['acc_z'] for d in self.raw_data]
        raw_mag = [d['acc_magnitude'] for d in self.raw_data]
        linear_x = [d['linear_x'] for d in self.raw_data]
        linear_y = [d['linear_y'] for d in self.raw_data]
        linear_z = [d['linear_z'] for d in self.raw_data]
        linear_mag = [d['linear_magnitude'] for d in self.raw_data]

        # 创建输出目录
        if output_dir is None:
            base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
            output_dir = os.path.join(os.path.dirname(self.csv_file), "plots", base_name + "_original")
        os.makedirs(output_dir, exist_ok=True)

        dpi = 300

        # 图1: 三轴原始加速度
        fig1, ax1 = plt.subplots(figsize=(14, 5))
        ax1.plot(timestamps, raw_x, 'r-', linewidth=0.5, label='X轴')
        ax1.plot(timestamps, raw_y, 'g-', linewidth=0.5, label='Y轴')
        ax1.plot(timestamps, raw_z, 'b-', linewidth=0.5, label='Z轴')
        ax1.set_xlabel('时间 (秒)')
        ax1.set_ylabel('加速度 (g)')
        ax1.set_title('原始三轴加速度')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file1 = os.path.join(output_dir, '01_raw_3axis.png')
        plt.savefig(output_file1, dpi=dpi)
        logger.info(f"图表已保存: {output_file1}")
        plt.close()

        # 图2: 原始加速度幅值
        fig2, ax2 = plt.subplots(figsize=(14, 5))
        ax2.plot(timestamps, raw_mag, 'b-', linewidth=0.8)
        ax2.axhline(y=1.0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5, label='1g')
        ax2.set_xlabel('时间 (秒)')
        ax2.set_ylabel('加速度 (g)')
        ax2.set_title('原始加速度幅值')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file2 = os.path.join(output_dir, '02_raw_magnitude.png')
        plt.savefig(output_file2, dpi=dpi)
        logger.info(f"图表已保存: {output_file2}")
        plt.close()

        # 图3: 三轴线性加速度
        fig3, ax3 = plt.subplots(figsize=(14, 5))
        ax3.plot(timestamps, linear_x, 'r-', linewidth=0.5, label='X轴')
        ax3.plot(timestamps, linear_y, 'g-', linewidth=0.5, label='Y轴')
        ax3.plot(timestamps, linear_z, 'b-', linewidth=0.5, label='Z轴')
        ax3.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
        ax3.set_xlabel('时间 (秒)')
        ax3.set_ylabel('加速度 (g)')
        ax3.set_title('重力去除后三轴线性加速度')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file3 = os.path.join(output_dir, '03_linear_3axis.png')
        plt.savefig(output_file3, dpi=dpi)
        logger.info(f"图表已保存: {output_file3}")
        plt.close()

        # 图4: 线性加速度幅值
        fig4, ax4 = plt.subplots(figsize=(14, 5))
        ax4.plot(timestamps, linear_mag, 'r-', linewidth=0.8)
        ax4.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5, label='零线')
        ax4.set_xlabel('时间 (秒)')
        ax4.set_ylabel('加速度 (g)')
        ax4.set_title('重力去除后线性加速度幅值')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        plt.tight_layout()
        output_file4 = os.path.join(output_dir, '04_linear_magnitude.png')
        plt.savefig(output_file4, dpi=dpi)
        logger.info(f"图表已保存: {output_file4}")
        plt.close()

        logger.info(f"所有图表已保存到目录: {output_dir}")

        self.analyze_statistics(self.raw_data)


def find_latest_gravity_file(data_dir='data/gravity'):
    """查找最新的重力数据文件"""
    if not os.path.exists(data_dir):
        return None

    csv_files = [f for f in os.listdir(data_dir) if f.endswith('.csv')]
    if not csv_files:
        return None

    csv_files.sort(key=lambda f: os.path.getmtime(os.path.join(data_dir, f)), reverse=True)
    return os.path.join(data_dir, csv_files[0])


def main():
    # 从config读取默认参数
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        import config
        default_alpha = config.GRAVITY_REMOVER_CONFIG.get("filter_alpha", 0.3)
        default_window = config.GRAVITY_REMOVER_CONFIG.get("filter_window", 5)
        default_t_max = config.STEP_CONFIG.get("t_max", 0.15)
        default_t_min = config.STEP_CONFIG.get("t_min", -0.05)
    except:
        default_alpha = 0.3
        default_window = 5
        default_t_max = 0.15
        default_t_min = -0.05

    parser = argparse.ArgumentParser(description='重力去除分析器')
    parser.add_argument('input', nargs='?', help='输入CSV文件')
    parser.add_argument('--alpha', type=float, default=default_alpha, help=f'滤波系数 (默认{default_alpha})')
    parser.add_argument('--window', type=int, default=default_window, help=f'滑动窗口大小 (默认{default_window})')
    parser.add_argument('--t-max', type=float, default=default_t_max, help=f'步数检测波峰阈值 (默认{default_t_max})')
    parser.add_argument('--t-min', type=float, default=default_t_min, help=f'步数检测波谷阈值 (默认{default_t_min})')
    parser.add_argument('-o', '--output', help='输出图片文件')
    parser.add_argument('--original', action='store_true', help='只绘制原始数据图')

    args = parser.parse_args()

    # 自动查找最新文件
    input_file = args.input
    if not input_file:
        input_file = find_latest_gravity_file()
        if not input_file:
            logger.error("未找到数据文件，请先运行 gravity_collector.py 采集数据")
            sys.exit(1)

    logger.info(f"分析文件: {input_file}")
    logger.info(f"参数: alpha={args.alpha}, window={args.window}")

    analyzer = GravityAnalyzer(input_file, step_t_max=args.t_max, step_t_min=args.t_min)

    if not analyzer.load_data():
        sys.exit(1)

    if args.original:
        analyzer.plot_original(args.output)
    else:
        analyzer.plot_comparison(args.alpha, args.window, args.output, t_max=args.t_max, t_min=args.t_min)


if __name__ == '__main__':
    main()
