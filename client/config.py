# -*- coding: UTF-8 -*-
"""
配置文件
包含服务器配置、硬件引脚定义、算法参数等
"""

# ==================== 服务器配置 ====================
SERVER_URL = "http://your-server-ip:5000"
UPDATE_INTERVAL = 2

# ==================== 日志配置 ====================
LOG_CONFIG = {
    'log_level': 'INFO',       # 日志级别: DEBUG, INFO, WARNING, ERROR
    'log_dir': 'logs',          # 日志文件目录
    'log_to_file': True,        # 是否输出到文件
    'log_to_console': True,     # 是否输出到控制台
}

# ==================== 调试配置 ====================
DEBUG_ENABLED = False
DEBUG_DIR = "debug_data"

# ==================== 硬件引脚定义 ====================
# 注意: 字符串格式，将在运行时转换为Pin对象
PIN_DHT11 = "P23"
PIN_BUTTON = "P24"
PIN_KNOB = "P22"
PIN_LED = "P21"
LED_COUNT = 19

# ==================== 模式定义 ====================
MODE_LIFE = 0
MODE_SPORT = 1
MODE_MEETING = 2

# ==================== 步数检测器参数 ====================
# 基于论文：《基于腰部MEMS加速度计的多阈值步数检测算法》作者：蒋博、付乐乐
STEP_CONFIG = {
    "t_max": 0.12,             # 波峰阈值 (g)
    "t_min": -0.06,            # 波谷阈值 (g)
    "window_size": 7,          # 滑动窗口大小
    "min_interval_ms": 250,   # 最小步间隔250ms
    "max_interval_ms": 2000,   # 最大步间隔2秒
}

# ==================== 重力去除器参数 ====================
# 基于论文：《基于MEMS六轴传感器的上肢运动识别系统》作者：胡成全等
GRAVITY_REMOVER_CONFIG = {
    "filter_alpha": 0.22,     # 滤波系数 (0-1)，越小响应越快，建议0.1-0.3
    "filter_window": 3,       # 滑动平均窗口大小，越大越平滑，建议3-5
}

# ==================== 姿态检测器参数 ====================
# 基于论文：《基于加速度传感器的人体姿态识别研究-李毅》
#          《基于加速度传感器的人体运动姿态识别算法研究-董雪薇》
POSTURE_CONFIG = {
    "sit_pitch_change": 18,
    "sit_roll_change": 23,
    "motion_threshold": 0.05,
    "still_threshold": 0.02,
    "stable_window": 15,
    "hysteresis_count": 5,
    # Y轴分量法参数
    "y_baseline_samples": 30,
    "y_sit_threshold": 0.18,
    "y_stand_threshold": 0.10,
    # 姿态角计算参数
    "window_size": 10,
    "lowpass_alpha": 0.3,
}

# ==================== 跌倒检测器参数 ====================
# 基于论文：《一种基于三轴加速度的跌倒检测方法》作者：李争
FALL_CONFIG = {
    "ra_threshold": 1.85,       # 合加速度阈值 (g)
    "sa_threshold": 2.2,        # 加速度变化率阈值
    "energy_threshold": 3.5,     # 加速度能量阈值
    "dip_threshold": 0.3,       # 方向变化阈值
    "window_size": 50,          # 分析窗口大小
}

# ==================== 姿态角计算器参数 ====================
ATTITUDE_CONFIG = {
    "window_size": 10,
    "lowpass_alpha": 0.3,
}

# ==================== 运动统计参数 ====================
CARBON_PER_STEP = 0.03
# 步频阈值（步/分钟）
STEP_FREQ_SLOW = 110      # 慢走：<110步/分钟
STEP_FREQ_NORMAL = 140    # 行走：110~140步/分钟
# 步长（米）
STEP_LENGTH_SLOW = 0.5
STEP_LENGTH_NORMAL = 0.65
STEP_LENGTH_FAST = 0.8

# ==================== 久坐提醒参数 ====================
DEFAULT_SITTING_REMIND_DURATION = 3600
