# -*- coding: UTF-8 -*-
"""
数据采集器

50Hz采样 + 重力去除
- 采样率: 50Hz
- 输出: 原始加速度和线性加速度

论文算法:
1. 《基于MEMS六轴传感器的上肢运动识别系统》(胡成全等)
2. 《基于腰部MEMS加速度计的多阈值步数检测算法》(蒋博等)
"""

import time
import csv
import os
import sys
import signal

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入日志模块
from utils.logger import get_logger
logger = get_logger('tools.data_collector')

# 输出目录
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")

# 全局标志
running = True


def signal_handler(_, __):
    """处理Ctrl+C信号"""
    global running
    logger.warning("收到停止信号，正在停止...")
    running = False


def ensure_output_dir():
    """确保输出目录存在"""
    global OUTPUT_DIR
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        logger.info(f"创建数据目录: {OUTPUT_DIR}")


def get_output_filename(prefix="walk_data"):
    """获取带时间戳的输出文件名"""
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(OUTPUT_DIR, f"{prefix}_{timestamp}.csv")


def collect_data(duration_seconds=None):
    """
    采集数据 - 50Hz采样 + 重力去除

    参数:
        duration_seconds: 采集时长(秒)
    """
    global running
    running = True

    ensure_output_dir()
    output_file = get_output_filename()
    logger.info(f"输出文件: {output_file}")

    # 使用主程序的高频采样器
    from sensors.high_freq_sampler import HighFrequencySampler

    # 创建采样器：50Hz采样 + 重力去除
    sampler = HighFrequencySampler(sample_rate=50)

    # 启动采样
    try:
        if not sampler.start():
            logger.error("采样器启动失败")
            return 0
    except RuntimeError as e:
        logger.error(f"采样器启动失败: {e}")
        return 0

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # CSV字段 - 完整数据（兼容重力去除和步数检测分析）
    fieldnames = [
        'timestamp',
        'sample_idx',
        # 原始加速度
        'acc_x', 'acc_y', 'acc_z', 'acc_magnitude',
        # 重力加速度（用于重力去除分析）
        'gravity_x', 'gravity_y', 'gravity_z',
        # 线性加速度（用于步数检测）
        'linear_x', 'linear_y', 'linear_z',
    ]

    sample_count = 0
    last_saved_idx = -1
    start_time = time.time()

    try:
        with open(output_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            logger.info("开始采集 (50Hz采样 + 重力去除)...")
            logger.info("预处理: 使用主程序 HighFrequencySampler + GravityRemover")
            logger.info("按 Ctrl+C 停止采集")

            while running:
                # 获取检测数据
                detect_data = sampler.get_output_buffer()

                # 保存所有新的检测数据
                for d in detect_data:
                    if d['sample_idx'] > last_saved_idx:
                        # 获取重力加速度
                        gravity_remover = sampler._gravity_remover
                        if gravity_remover:
                            gx, gy, gz = gravity_remover.get_gravity()
                        else:
                            gx, gy, gz = 0, 0, 0

                        writer.writerow({
                            'timestamp': d['timestamp'],
                            'sample_idx': d['sample_idx'],
                            'acc_x': d['ax'],
                            'acc_y': d['ay'],
                            'acc_z': d['az'],
                            'acc_magnitude': d['acc_mag'],
                            'gravity_x': gx,
                            'gravity_y': gy,
                            'gravity_z': gz,
                            'linear_x': d['linear_x'],
                            'linear_y': d['linear_y'],
                            'linear_z': d['linear_z'],
                            'linear_magnitude': d['linear_mag'],
                        })
                        sample_count += 1
                        last_saved_idx = d['sample_idx']

                # 打印进度
                elapsed = time.time() - start_time
                if sample_count % 250 == 0 and sample_count > 0:
                    stats = sampler.get_stats()
                    logger.info(f"已采集 {sample_count} 样本({stats['actual_sample_rate']:.1f}Hz), 耗时 {elapsed:.1f}秒")

                # 检查时长
                if duration_seconds is not None:
                    if elapsed >= duration_seconds:
                        running = False

                time.sleep(0.01)

    except Exception as e:
        logger.error(f"采集错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 停止采样器
        sampler.stop()

        elapsed = time.time() - start_time
        stats = sampler.get_stats()

        logger.info("完成!")
        logger.info(f"采样数据: {stats['sample_count']} 样本")
        logger.info(f"实际采样率: {stats['actual_sample_rate']:.1f} Hz")
        logger.info(f"数据已保存到: {output_file}")

    return sample_count


def main():
    import argparse

    parser = argparse.ArgumentParser(description='步态数据采集器 - 50Hz采样+重力去除')
    parser.add_argument('-d', '--duration', type=int, default=None,
                        help='采集时长(秒)，默认无限')

    args = parser.parse_args()

    logger.info("=" * 60)
    logger.info("步态数据采集器 - 50Hz采样 + 重力去除")
    logger.info("=" * 60)
    logger.info("预处理: HighFrequencySampler + GravityRemover")

    collect_data(args.duration)


if __name__ == '__main__':
    main()
