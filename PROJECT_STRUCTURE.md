# 项目结构说明

本文档说明智能运动腰带系统的文件组织结构。

## 目录树

```
smart-sports-anklet/
│
├── client/                      # 客户端程序（行空板端）
│   ├── main.py                 # 主程序入口
│   ├── config.py               # 客户端配置
│   ├── sensors/                # 传感器模块
│   │   ├── __init__.py
│   │   ├── icm20689.py        # ICM20689加速度传感器读取（50Hz）
│   │   ├── attitude.py         # 姿态角计算器
│   │   ├── step_detector.py   # 步数检测器
│   │   ├── posture_detector.py# 坐姿检测器
│   │   ├── fall_detector.py   # 跌倒检测器
│   │   └── gravity_remover.py # 重力去除
│   ├── tools/                  # 数据分析工具
│   │   ├── data_analyzer.py
│   │   ├── data_collector.py
│   │   ├── data_plotter.py
│   │   └── step_counter.py
│   └── utils/                  # 工具模块
│       ├── __init__.py
│       ├── debug.py            # 调试日志
│       └── helpers.py          # 辅助函数
│
├── server/                     # 服务器程序（PC端）
│   ├── server.py               # Flask服务器
│   ├── control.html            # Web控制界面
│   └── data/                   # 数据存储目录（运行时创建）
│
├── docs/                       # 文档和参考资料
│   └── 论文/                   # 参考文献
│
├── start_server.bat           # Windows服务器启动脚本
├── start_server.sh            # Linux服务器启动脚本
│
├── requirements.txt           # Python依赖
├── LICENSE                   # MIT开源许可证
├── README.md                 # 项目主文档
├── QUICKSTART.md            # 快速开始指南
└── PROJECT_STRUCTURE.md      # 本文档
```

---

## 核心文件说明

### 客户端模块 (client/)

| 文件 | 功能描述 |
|------|---------|
| main.py | 主程序入口，处理3种模式（生活/运动/会议） |
| config.py | 客户端配置参数 |
| sensors/icm20689.py | 直接读取ICM20689，实现50Hz高频采样 |
| sensors/step_detector.py | 基于多阈值算法的步数检测 |
| sensors/posture_detector.py | 基于Y轴分量和Roll角的坐姿检测 |
| sensors/fall_detector.py | 基于多特征融合的跌倒检测 |
| sensors/attitude.py | 姿态角计算（俯仰角/翻滚角） |
| sensors/gravity_remover.py | 基于低通滤波的重力去除 |
| utils/debug.py | 调试数据记录器 |
| utils/helpers.py | 配速计算等辅助函数 |

### 服务器模块 (server/)

| 文件 | 功能描述 |
|------|---------|
| server.py | Flask Web服务器，提供API接口 |
| control.html | 响应式Web控制界面 |
| data/ | 运动记录存储目录 |

---

## 启动说明

### 1. 启动服务器（PC端）

```bash
# Windows
start_server.bat

# Linux/Mac
chmod +x start_server.sh
./start_server.sh
```

服务器启动后访问：http://localhost:5000

### 2. 启动客户端（行空板端）

```bash
cd client
python main.py
```

---

## 配置文件说明

### 客户端配置 (client/config.py)

```python
# 服务器地址
SERVER_URL = "http://your-server-ip:5000"

# 硬件引脚
PIN_DHT11 = "P22"      # 温湿度传感器
PIN_TOUCH = "P24"      # 按钮
PIN_KNOB = "P23"       # 旋钮
PIN_LED = "P21"         # LED灯带

# 模式定义
MODE_LIFE = 0      # 生活模式
MODE_SPORT = 1     # 运动模式
MODE_MEETING = 2   # 会议模式
```

---

## 数据文件

### 服务器数据 (server/data/)

| 文件 | 说明 |
|------|------|
| emergency.json | 紧急事件记录 |
| history.json | 历史运动数据 |
| sport_records.json | 运动记录详情 |
| settings.json | 系统设置 |

### 客户端调试数据 (debug_data/)

| 文件 | 说明 |
|------|------|
| debug_*.csv | 调试数据文件（启用DEBUG_ENABLED时生成） |
