# -*- coding: UTF-8 -*-
"""
Flask服务器程序 - 运动腰带系统
提供REST API和Web界面
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from datetime import datetime, timedelta
import json
import threading
import time
import os

# 数据保存调度器
last_save_time = 0
save_lock = threading.Lock()
SAVE_INTERVAL = 30  # 30秒保存一次

def atomic_save(filepath, data):
    """原子性保存：先写临时文件，再重命名"""
    temp_path = filepath + ".tmp"
    try:
        with open(temp_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(temp_path, filepath)
    except Exception as e:
        print(f"保存失败 {filepath}: {e}")
        if os.path.exists(temp_path):
            try:
                os.remove(temp_path)
            except:
                pass

def schedule_save():
    """定时保存调度器"""
    global last_save_time
    if time.time() - last_save_time > SAVE_INTERVAL:
        save_all_data()
        last_save_time = time.time()

def save_all_data():
    """保存所有数据"""
    with save_lock:
        try:
            atomic_save(HISTORY_FILE, sport_history)
            atomic_save(EMERGENCY_FILE, emergency_records)
            atomic_save(SPORT_RECORDS_FILE, sport_records)
            atomic_save(SETTINGS_FILE, {"sitting_remind_duration": sitting_remind_duration})
            print(f"[数据] 已保存")
        except Exception as e:
            print(f"[数据] 保存失败: {e}")

app = Flask(__name__)
CORS(app)

# 全局数据存储
device_status = {
    "mode": 0,
    "temperature": 25.0,
    "humidity": 50.0,
    "brightness": 100,
    "posture": "unknown",
    "pace": 0,
    "pace_str": "--'--\"",
    "step": 0,
    "carbon_reduce": 0,
    "emergency": False,
    "sport_time_today": 0,
    "activity_hours": [],
    "last_update": None,
    "online": False,
    "sitting_duration": 0,
    "sport_duration": 0,
    "message_showing": False
}

# 控制命令队列
control_commands = []
command_lock = threading.Lock()

# 紧急记录
emergency_records = []

# 历史运动数据
sport_history = {}

# 运动记录
sport_records = []

# 设备状态跟踪
device_last_step = 0  # 上次上报的步数

# 久坐提醒设置（秒）
sitting_remind_duration = 3600

# 数据文件路径
DATA_DIR = "data"
EMERGENCY_FILE = os.path.join(DATA_DIR, "emergency.json")
HISTORY_FILE = os.path.join(DATA_DIR, "history.json")
SPORT_RECORDS_FILE = os.path.join(DATA_DIR, "sport_records.json")
SETTINGS_FILE = os.path.join(DATA_DIR, "settings.json")

# 确保数据目录存在
os.makedirs(DATA_DIR, exist_ok=True)

# ==================== 数据持久化 ====================
def load_data():
    """加载历史数据"""
    global emergency_records, sport_history, sport_records, sitting_remind_duration
    
    try:
        if os.path.exists(EMERGENCY_FILE):
            with open(EMERGENCY_FILE, 'r', encoding='utf-8') as f:
                emergency_records = json.load(f)
    except:
        emergency_records = []
    
    try:
        if os.path.exists(HISTORY_FILE):
            with open(HISTORY_FILE, 'r', encoding='utf-8') as f:
                sport_history = json.load(f)
    except:
        sport_history = {}
    
    try:
        if os.path.exists(SPORT_RECORDS_FILE):
            with open(SPORT_RECORDS_FILE, 'r', encoding='utf-8') as f:
                sport_records = json.load(f)
    except:
        sport_records = []
    
    try:
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r', encoding='utf-8') as f:
                settings = json.load(f)
                sitting_remind_duration = settings.get("sitting_remind_duration", 3600)
    except:
        sitting_remind_duration = 3600

def save_emergency():
    """保存紧急记录"""
    try:
        with open(EMERGENCY_FILE, 'w', encoding='utf-8') as f:
            json.dump(emergency_records, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"保存紧急记录失败: {e}")

def save_history():
    """保存历史数据"""
    try:
        with open(HISTORY_FILE, 'w', encoding='utf-8') as f:
            json.dump(sport_history, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"保存历史数据失败: {e}")

def save_sport_records():
    """保存运动记录"""
    try:
        with open(SPORT_RECORDS_FILE, 'w', encoding='utf-8') as f:
            json.dump(sport_records, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"保存运动记录失败: {e}")

def save_settings():
    """保存设置"""
    try:
        settings = {"sitting_remind_duration": sitting_remind_duration}
        with open(SETTINGS_FILE, 'w', encoding='utf-8') as f:
            json.dump(settings, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"保存设置失败: {e}")

# ==================== API端点 ====================

@app.route('/')
def index():
    """返回控制页面"""
    return send_from_directory(os.path.dirname(__file__), 'control.html')

@app.route('/api/status', methods=['POST'])
def update_status():
    """接收设备状态更新"""
    global device_status, sport_history, device_last_step

    try:
        data = request.json
        current_time = datetime.now()

        # 获取当前步数
        current_step = data.get("step", 0)

        # 计算步数增量
        step_increment = max(0, current_step - device_last_step)

        # 计算碳排放增量（仅在运动模式）
        current_mode = data.get("mode", 0)
        carbon_increment = 0

        # 只有在运动模式(MODE_SPORT=1)时才累加碳排放
        if current_mode == 1 and step_increment > 0:
            # 验证增量合理性：单次上报超过20步可能异常，忽略
            if step_increment <= 20:
                carbon_increment = step_increment * 0.03
                device_status["carbon_reduce"] = round(
                    device_status["carbon_reduce"] + carbon_increment, 4
                )

        # 更新步数记录
        device_last_step = current_step

        # 更新设备状态
        device_status.update({
            "mode": current_mode,
            "temperature": data.get("temperature", device_status["temperature"]),
            "humidity": data.get("humidity", device_status["humidity"]),
            "brightness": data.get("brightness", device_status["brightness"]),
            "posture": data.get("posture", device_status["posture"]),
            "pace": data.get("pace", device_status["pace"]),
            "pace_str": data.get("pace_str", device_status["pace_str"]),
            "step": current_step,
            "carbon_reduce": device_status["carbon_reduce"],
            "emergency": data.get("emergency", device_status["emergency"]),
            "sport_time_today": data.get("sport_time_today", device_status["sport_time_today"]),
            "activity_hours": data.get("activity_hours", device_status["activity_hours"]),
            "sitting_duration": data.get("sitting_duration", device_status["sitting_duration"]),
            "sport_duration": data.get("sport_duration", device_status["sport_duration"]),
            "message_showing": data.get("message_showing", device_status["message_showing"]),
            "last_update": current_time.strftime("%Y-%m-%d %H:%M:%S"),
            "online": True
        })

        # 更新今日历史数据
        today = current_time.strftime("%Y-%m-%d")
        if today not in sport_history:
            sport_history[today] = {
                "sport_time": 0,
                "activity_hours": [],
                "step": 0,
                "carbon_reduce": 0
            }

        sport_history[today]["sport_time"] = device_status["sport_time_today"]
        sport_history[today]["activity_hours"] = device_status["activity_hours"]
        sport_history[today]["carbon_reduce"] = device_status["carbon_reduce"]
        sport_history[today]["step"] = current_step

        # 定期保存历史数据（使用新的调度器）
        schedule_save()

        # 处理紧急情况
        if data.get("emergency") and not device_status.get("emergency_recorded"):
            emergency_records.append({
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "message": "检测到摔倒，需要紧急救助",
                "location": "未知",
                "resolved": False
            })
            save_emergency()
            device_status["emergency_recorded"] = True
        elif not data.get("emergency"):
            device_status["emergency_recorded"] = False
        
        # 返回待执行的命令
        with command_lock:
            commands = control_commands.copy()
            control_commands.clear()
        
        return jsonify({
            "status": "ok",
            "commands": commands,
            "sitting_remind_duration": sitting_remind_duration
        })
        
    except Exception as e:
        print(f"更新状态错误: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/status', methods=['GET'])
def get_status():
    """获取设备状态"""
    # 只从历史数据计算总碳排放
    carbon_from_history = 0

    days = request.args.get('days', 7, type=int)
    today = datetime.now()

    for i in range(days):
        date = (today - timedelta(days=i)).strftime("%Y-%m-%d")
        if date in sport_history:
            carbon_from_history += sport_history[date].get("carbon_reduce", 0)

    # 设备在线检测
    if device_status["last_update"]:
        try:
            last_time = datetime.strptime(device_status["last_update"], "%Y-%m-%d %H:%M:%S")
            if datetime.now() - last_time > timedelta(seconds=10):
                device_status["online"] = False
        except:
            device_status["online"] = False

    device_status['carbon_reduce_all'] = round(carbon_from_history, 4)

    return jsonify(device_status)

@app.route('/api/control', methods=['POST'])
def send_control():
    """发送控制命令"""
    try:
        data = request.json
        command = data.get("command")
        
        with command_lock:
            control_commands.append(data)
        
        print(f"收到控制命令: {command}")
        return jsonify({"status": "ok"})
        
    except Exception as e:
        print(f"发送控制命令错误: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/emergency', methods=['GET'])
def get_emergency_records():
    """获取紧急记录"""
    return jsonify(emergency_records)

@app.route('/api/emergency/<int:index>', methods=['PUT'])
def resolve_emergency(index):
    """标记紧急情况已解决"""
    try:
        if 0 <= index < len(emergency_records):
            emergency_records[index]["resolved"] = True
            save_emergency()
            return jsonify({"status": "ok"})
        return jsonify({"status": "error", "message": "记录不存在"}), 404
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/history', methods=['GET'])
def get_history():
    """获取历史数据"""
    days = request.args.get('days', 7, type=int)
    
    result = {}
    today = datetime.now()
    
    for i in range(days):
        date = (today - timedelta(days=i)).strftime("%Y-%m-%d")
        if date in sport_history:
            result[date] = sport_history[date]
        else:
            result[date] = {"sport_time": 0, "activity_hours": []}
    
    return jsonify(result)

@app.route('/api/message', methods=['POST'])
def send_message():
    """向设备发送消息"""
    try:
        data = request.json
        message = data.get("message", "")
        
        with command_lock:
            control_commands.append({
                "command": "message",
                "content": message
            })
        
        return jsonify({"status": "ok"})
        
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/sport_records', methods=['GET'])
def get_sport_records():
    """获取运动记录"""
    return jsonify(sport_records)

@app.route('/api/sport_records', methods=['POST'])
def add_sport_record():
    """添加运动记录"""
    try:
        data = request.json
        sport_records.append(data)
        save_sport_records()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/api/sync_records', methods=['POST'])
def sync_records():
    """批量同步运动记录（离线模式切换到在线时使用）"""
    try:
        data = request.json
        records = data.get("records", [])

        if not records:
            return jsonify({"status": "ok", "synced_count": 0})

        synced_count = 0
        for record in records:
            sport_records.append(record)
            synced_count += 1

        save_sport_records()
        print(f"[同步] 批量接收 {synced_count} 条运动记录")

        return jsonify({
            "status": "ok",
            "synced_count": synced_count
        })
    except Exception as e:
        print(f"批量同步错误: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/api/sync_emergency', methods=['POST'])
def sync_emergency():
    """批量同步紧急记录"""
    try:
        data = request.json
        records = data.get("records", [])

        if not records:
            return jsonify({"status": "ok", "synced_count": 0})

        synced_count = 0
        for record in records:
            emergency_records.append(record)
            synced_count += 1

        save_emergency()
        print(f"[同步] 批量接收 {synced_count} 条紧急记录")

        return jsonify({
            "status": "ok",
            "synced_count": synced_count
        })
    except Exception as e:
        print(f"批量同步错误: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/settings', methods=['GET'])
def get_settings():
    """获取设置"""
    return jsonify({"sitting_remind_duration": sitting_remind_duration})

@app.route('/api/settings', methods=['POST'])
def update_settings():
    """更新设置"""
    global sitting_remind_duration
    try:
        data = request.json
        sitting_remind_duration = data.get("sitting_remind_duration", sitting_remind_duration)
        save_settings()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# ==================== 启动服务 ====================
def check_offline():
    """定期检查设备在线状态"""
    while True:
        time.sleep(5)
        if device_status["last_update"]:
            try:
                last_time = datetime.strptime(device_status["last_update"], "%Y-%m-%d %H:%M:%S")
                if datetime.now() - last_time > timedelta(seconds=10):
                    if device_status["online"]:
                        device_status["online"] = False
                        print("设备已离线")
            except:
                pass

if __name__ == "__main__":
    print("=" * 50)
    print("运动腰带Flask服务器")
    print("=" * 50)
    
    load_data()
    print(f"✓ 加载历史数据: {len(emergency_records)}条紧急记录")
    print(f"✓ 加载运动记录: {len(sport_records)}条")
    
    offline_thread = threading.Thread(target=check_offline, daemon=True)
    offline_thread.start()
    
    print("✓ 服务器启动在 http://0.0.0.0:5000")
    print("✓ 控制页面: http://localhost:5000")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)