import sys
import os
import ctypes
import threading
import time
from datetime import datetime
import collections
import json
import re
import csv
import shutil
import subprocess

import psutil

# .NETランタイムのインポートはmainブロックで行う

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QProgressBar, QComboBox, QCheckBox,
    QSystemTrayIcon, QMenu
)
from PyQt6.QtGui import QFont, QPainter, QPen, QColor, QPainterPath, QIcon, QAction
from PyQt6.QtCore import QTimer, Qt, QPointF, QByteArray


# ==========================
# 実行パス／DLLパスの設定c
# ==========================
def base_dir():
    """
    IDE実行時: .../PCMonitoringTool
    EXE実行時: .../dist/PC_Performance_Monitor
    """
    return os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) \
        else os.path.dirname(os.path.abspath(__file__))


# 常に 'DLLs' サブフォルダを参照する
DLLS_DIR = os.path.join(base_dir(), 'DLLs')

# グローバル変数
computer = None
Hardware = None


# DLLからプロセス情報を受け取るための構造体
class PROCINFO(ctypes.Structure):
    _fields_ = [
        ("pid", ctypes.c_ulong),
        ("threads", ctypes.c_ulong),
        ("cpu", ctypes.c_double),
        ("ram", ctypes.c_size_t),
        ("swap", ctypes.c_size_t),
        ("name", ctypes.c_char * 260)
    ]


# ==========================
# C++ DLLのロードと関数定義
# ==========================
try:
    # EXE実行時(_internal)にDLLの依存関係(getcoreinfo等)を見つけるため
    if getattr(sys, 'frozen', False):
        if hasattr(os, 'add_dll_directory'):
            try:
                os.add_dll_directory(DLLS_DIR)
            except Exception:
                pass
        else:
            os.environ['PATH'] = DLLS_DIR + os.pathsep + os.environ['PATH']

    dll_pc_path = os.path.join(DLLS_DIR, "pcmonitor.dll")
    dll_core_path = os.path.join(DLLS_DIR, "getcoreinfo.dll")

    dll_pc = ctypes.CDLL(dll_pc_path)
    dll_core = ctypes.CDLL(dll_core_path)

    # データ取得関数の定義
    dll_pc.pm_log_system_fast.argtypes = [ctypes.c_char_p]
    dll_pc.pm_log_system_fast.restype = ctypes.c_int
    dll_pc.pm_log_detail_active.argtypes = [ctypes.c_char_p]
    dll_pc.pm_log_detail_active.restype = ctypes.c_int
    dll_pc.pm_log_snapshot.argtypes = [ctypes.c_char_p]
    dll_pc.pm_log_snapshot.restype = ctypes.c_int
    dll_pc.pm_get_memory_totals.argtypes = [
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)
    ]
    dll_pc.pm_get_memory_totals.restype = ctypes.c_int
    dll_pc.pm_get_top_processes.argtypes = [ctypes.c_int, ctypes.POINTER(PROCINFO), ctypes.POINTER(PROCINFO)]
    dll_pc.pm_get_top_processes.restype = ctypes.c_int

    # ヘッダー書き込み関数の定義
    dll_pc.pm_log_fast_write_header.argtypes = [ctypes.c_char_p]
    dll_pc.pm_log_fast_write_header.restype = ctypes.c_int
    dll_pc.pm_detail_active_write_header.argtypes = [ctypes.c_char_p]
    dll_pc.pm_detail_active_write_header.restype = ctypes.c_int
    dll_pc.pm_snapshot_write_header.argtypes = [ctypes.c_char_p]
    dll_pc.pm_snapshot_write_header.restype = ctypes.c_int

    dll_core.get_core_count.restype = ctypes.c_int
    dll_core.get_core_usage.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    dll_core.get_core_usage.restype = ctypes.c_int

except (FileNotFoundError, OSError) as e:
    # ここでエラーが出た場合はコンソール（またはGUIなし終了）になるが、
    # 本番環境ではログファイル出力機構を別途設けない限りユーザーには見えない。
    sys.exit(1)


# ==========================
# LibreHardwareMonitor の初期化
# ==========================
def initialize_hardware_monitor():
    global computer, Hardware
    try:
        from LibreHardwareMonitor import Hardware as LHM_Hardware
        Hardware = LHM_Hardware
        computer = Hardware.Computer()
        computer.IsCpuEnabled = True
        computer.Open()
    except Exception:
        computer = None


def get_cpu_info():
    if not computer:
        return {'core_temps': {}, 'core_clocks': {}, 'core_voltages': {}, 'package_temp': None, 'tjmax': None,
                'package_power': None, 'is_throttling': False}

    info = {'package_temp': None, 'tjmax': None, 'package_power': None, 'is_throttling': False}
    core_temp_data, core_clock_data, core_voltage_data = {}, {}, {}
    core_num_re = re.compile(r'CPU Core #(\d+)')
    sensors_temp, sensors_clock, sensors_voltage = {}, {}, {}
    throttle_sensor_keywords = ["Throttling", "Limit", "Exceeded"]

    for hw in computer.Hardware:
        if hw.HardwareType == Hardware.HardwareType.Cpu:
            hw.Update()
            for sensor in hw.Sensors:
                if sensor.SensorType == Hardware.SensorType.Temperature:
                    sensors_temp[sensor.Name] = sensor.Value
                elif sensor.SensorType == Hardware.SensorType.Clock:
                    sensors_clock[sensor.Name] = sensor.Value
                elif sensor.SensorType == Hardware.SensorType.Power and "CPU Package" in sensor.Name:
                    info['package_power'] = sensor.Value
                elif sensor.SensorType == Hardware.SensorType.Voltage:
                    sensors_voltage[sensor.Name] = sensor.Value
                elif any(keyword in sensor.Name for keyword in
                         throttle_sensor_keywords) and sensor.Value is not None and sensor.Value > 0:
                    info['is_throttling'] = True

    for name, value in sensors_temp.items():
        if value is None: continue
        if name == 'CPU Package': info['package_temp'] = value; continue
        match = core_num_re.match(name)
        if match and "Distance to TjMax" not in name:
            core_index = int(match.group(1)) - 1
            core_temp_data[core_index] = value
            dist_name = f'CPU Core #{match.group(1)} Distance to TjMax'
            if dist_name in sensors_temp and sensors_temp[dist_name] is not None:
                if info['tjmax'] is None: info['tjmax'] = value + sensors_temp[dist_name]

    for name, value in sensors_clock.items():
        if value is None: continue
        match = core_num_re.match(name)
        if match: core_clock_data[int(match.group(1)) - 1] = value

    for name, value in sensors_voltage.items():
        if value is None: continue
        match = core_num_re.match(name)
        if match: core_voltage_data[int(match.group(1)) - 1] = value

    info.update({'core_temps': core_temp_data, 'core_clocks': core_clock_data, 'core_voltages': core_voltage_data})
    return info


# ==========================
# バッファ付き CSV 書き出し（Process_Logger-Prototype から移植）
# ==========================
class BufferedLogger:
    """行をためておき、一定数たまったらまとめて書き出す。ディスクへの書き込み回数を減らす。"""

    def __init__(self, filepath, headers, buffer_size=50):
        self.filepath = filepath
        self.buffer_size = buffer_size
        self.buffer = []
        self.lock = threading.Lock()
        try:
            with open(self.filepath, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(headers)
        except IOError as e:
            print(f"BufferedLogger header error: {e}")

    def add_row(self, row):
        with self.lock:
            self.buffer.append(row)
            if len(self.buffer) >= self.buffer_size:
                self._flush()

    def _flush(self):
        if not self.buffer:
            return
        try:
            with open(self.filepath, 'a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerows(self.buffer)
            self.buffer.clear()
        except IOError as e:
            print(f"BufferedLogger flush error: {e}")

    def force_flush(self):
        with self.lock:
            self._flush()


# ==========================
# ディスク・ネットワーク・GPU の取得（Process_Logger-Prototype から移植）
# ==========================
def _disk_root():
    return os.environ.get('SystemDrive', 'C:') + '\\' if sys.platform.startswith('win') else '/'


class IoSampler:
    """psutil でディスク・ネットワークの速度を、nvidia-smi で GPU の状態を取る。"""

    GPU_INTERVAL = 5  # nvidia-smi は起動コストがあるため、間隔を空けて呼ぶ

    def __init__(self):
        self.lock = threading.Lock()
        self.last_time = time.time()
        self.last_net = psutil.net_io_counters()
        self.last_disk = psutil.disk_io_counters()
        self.latest = {'disk_pct': None, 'disk_r': 0.0, 'disk_w': 0.0, 'net_sent': 0.0, 'net_recv': 0.0,
                       'gpu_load': None, 'gpu_mem': None, 'gpu_temp': None}
        self.nvidia_smi = shutil.which('nvidia-smi')
        self.gpu_enabled = True
        if self.nvidia_smi:
            threading.Thread(target=self._gpu_loop, daemon=True).start()

    def sample(self):
        """ディスク・ネットワークを更新する（GUIのタイマーから呼ぶ）。"""
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        try:
            net = psutil.net_io_counters()
            disk = psutil.disk_io_counters()
            mb = 1024 * 1024
            values = {
                'disk_pct': psutil.disk_usage(_disk_root()).percent,
                'net_sent': (net.bytes_sent - self.last_net.bytes_sent) / mb / dt,
                'net_recv': (net.bytes_recv - self.last_net.bytes_recv) / mb / dt,
            }
            if disk and self.last_disk:
                values['disk_r'] = (disk.read_bytes - self.last_disk.read_bytes) / mb / dt
                values['disk_w'] = (disk.write_bytes - self.last_disk.write_bytes) / mb / dt
            self.last_net, self.last_disk, self.last_time = net, disk, now
            with self.lock:
                self.latest.update(values)
        except Exception as e:
            print(f"IoSampler error: {e}")

    def _gpu_loop(self):
        flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
        while True:
            if self.gpu_enabled:
                try:
                    out = subprocess.run(
                        [self.nvidia_smi, '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu',
                         '--format=csv,noheader,nounits'],
                        capture_output=True, text=True, timeout=5, creationflags=flags).stdout
                    load, mem_used, mem_total, temp = [float(v) for v in out.splitlines()[0].split(',')]
                    with self.lock:
                        self.latest.update({'gpu_load': load, 'gpu_temp': temp,
                                            'gpu_mem': mem_used / mem_total * 100 if mem_total else None})
                except Exception:
                    pass
            time.sleep(self.GPU_INTERVAL)

    def get(self):
        with self.lock:
            return dict(self.latest)


class TemperatureGraph(QWidget):
    def __init__(self, max_points=60):
        super().__init__()
        self.data = collections.deque(maxlen=max_points)
        self.setMinimumHeight(100)
        self.max_points = max_points
        self.last_temp = None

    def add_point(self, temp):
        if temp is not None and (self.last_temp is None or abs(self.last_temp - temp) > 0.1):
            self.data.append(temp)
            self.last_temp = temp
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#3B4252"))

        if len(self.data) < 2:
            painter.setPen(QColor("#D8DEE9"));
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for data...");
            return

        w, h, p = self.width(), self.height(), 10
        gw, gh = w - 2 * p, h - 2 * p
        min_t, max_t = 20.0, 110.0
        t_range = max_t - min_t

        painter.setPen(QPen(QColor("#4C566A"), 1, Qt.PenStyle.DashLine))
        for t in range(20, 111, 20): painter.drawLine(p, int(h - p - ((t - min_t) / t_range) * gh), w - p,
                                                      int(h - p - ((t - min_t) / t_range) * gh))

        last_temp = self.data[-1]
        color = "#88C0D0" if last_temp < 50 else "#EBCB8B" if last_temp < 80 else "#BF616A"
        painter.setPen(QPen(QColor(color), 2))

        points = [QPointF(p + (i / (self.max_points - 1)) * gw, h - p - ((temp - min_t) / t_range) * gh) for i, temp in
                  enumerate(self.data)]
        path = QPainterPath();
        path.moveTo(points[0])
        for point in points[1:]: path.lineTo(point)
        painter.drawPath(path)

        font = QFont("Segoe UI", 14, QFont.Weight.Bold);
        painter.setFont(font)
        painter.setPen(QColor("#ECEFF4"));
        painter.drawText(p, p + 20, f"{last_temp:.1f} °C")


class MonitorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.running = False
        self.core_count = dll_core.get_core_count()
        self.data_cache, self.last_logged_core_data, self.log_paths, self.log_controls = {}, {}, {}, {}
        self.data_lock = threading.Lock()
        self.dll_proc_lock = threading.Lock()
        self.core_log_buffer = []
        self.log_on_change = True
        self.io_sampler = IoSampler()
        self.io_logger = None
        self.force_quit = False

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_session_dir = os.path.join(base_dir(), "logs", ts)
        os.makedirs(self.log_session_dir, exist_ok=True)

        self.config_file = "config.json"

        try:
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("MyCompanyName.MyProductName.PCMonitor")
            self.setWindowIcon(QIcon(os.path.join(base_dir(), "app.ico")))
        except Exception:
            pass

        self.log_settings = {
            'fast': {'enabled': True, 'interval': 1, 'last_run': 0, 'func': dll_pc.pm_log_system_fast},
            'corelog': {'enabled': True, 'interval': 1, 'last_run': 0, 'func': self.log_core_details},
            'detail': {'enabled': True, 'interval': 2, 'last_run': 0, 'func': dll_pc.pm_log_detail_active},
            'snapshot': {'enabled': False, 'interval': 600, 'last_run': 0, 'func': dll_pc.pm_log_snapshot},
            'io': {'enabled': True, 'interval': 1, 'last_run': 0, 'func': self.log_io}
        }
        self.load_settings()
        self.io_sampler.gpu_enabled = self.config.get('gpu_enabled', True)
        self.core_info = self._initialize_core_info()
        self.initUI()
        self._setup_tray()
        self.update_gui()

        self.timer = QTimer(self);
        self.timer.timeout.connect(self.update_gui);
        self.timer.start(1500)
        self.process_timer = QTimer(self);
        self.process_timer.timeout.connect(self.update_top_processes);
        self.process_timer.start(2000)

        if self.config.get('start_on_launch', False): self.start_logging()

    def _initialize_core_info(self):
        if not computer: return [{'label': f"LP {i}", 'is_second_thread': False, 'physical_core_num': i + 1} for i in
                                 range(self.core_count)]

        load_sensors = []
        for hw in computer.Hardware:
            if hw.HardwareType == Hardware.HardwareType.Cpu:
                hw.Update()
                for sensor in hw.Sensors:
                    if sensor.SensorType == Hardware.SensorType.Load and "CPU Core #" in sensor.Name: load_sensors.append(
                        sensor.Name)

        load_sensors.sort(key=lambda s: [int(t) if t.isdigit() else t.lower() for t in re.split('([0-9]+)', s)])

        core_info = []
        re_core_thread = re.compile(r'CPU Core #(\d+)(?: Thread #(\d+))?')
        for i, name in enumerate(load_sensors):
            if i >= self.core_count: break
            match = re_core_thread.search(name)
            if not match: continue
            core_num, thread_num_str = int(match.group(1)), match.group(2)
            label, is_second = f"Core {core_num}", False
            if thread_num_str:
                thread_num = int(thread_num_str)
                label += f" / T{thread_num}"
                if thread_num == 2: is_second = True
            core_info.append({'label': label, 'is_second_thread': is_second, 'physical_core_num': core_num})

        while len(core_info) < self.core_count:
            idx = len(core_info)
            core_info.append({'label': f"LP {idx}", 'is_second_thread': False, 'physical_core_num': idx + 1})
        return core_info

    def initUI(self):
        self.setWindowTitle("🛰️ PC Performance Monitor")
        geometry = self.config.get('window_geometry')
        if geometry:
            self.restoreGeometry(QByteArray.fromHex(geometry.encode()))
        else:
            self.setGeometry(50, 50, 1800, 980)

        central_widget = QWidget();
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget);
        main_layout.setContentsMargins(5, 5, 5, 5);
        main_layout.setSpacing(5)

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.create_control_panel(), 35)
        top_layout.addWidget(self.create_system_status_panel(), 25)
        top_layout.addWidget(self.create_cpu_global_panel(), 40)

        core_panel = self.create_core_details_panel()
        bottom_panel = self.create_top_processes_panel()

        main_layout.addLayout(top_layout, 0)
        main_layout.addWidget(core_panel, 1)
        main_layout.addWidget(bottom_panel, 0)
        self.set_stylesheet()

    def create_panel(self, title, layout):
        group = QGroupBox(title);
        group.setLayout(layout);
        return group

    def create_control_panel(self):
        layout = QHBoxLayout()

        vbox = QVBoxLayout()
        self.start_btn = QPushButton("▶️ Start Logging");
        self.stop_btn = QPushButton("⏹️ Stop Logging")
        self.stop_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_logging);
        self.stop_btn.clicked.connect(self.stop_logging)
        vbox.addWidget(self.start_btn);
        vbox.addWidget(self.stop_btn);
        vbox.addStretch()
        layout.addLayout(vbox)

        grid = QGridLayout()
        log_types = {'fast': 'Fast System Log', 'corelog': 'Core Details Log', 'detail': 'Active Detail Log',
                     'snapshot': 'Periodic Snapshot', 'io': 'Disk / Net / GPU Log'}
        intervals = ["1", "2", "5", "10", "30", "60", "300", "600"]

        for i, (key, name) in enumerate(log_types.items()):
            grid.addWidget(QLabel(name), i, 0)

            checkbox = QCheckBox()
            combo = QComboBox()

            checkbox.setChecked(self.log_settings[key]['enabled'])
            combo.addItems(intervals)
            saved_interval = str(self.log_settings[key]['interval'])
            if combo.findText(saved_interval) != -1: combo.setCurrentText(saved_interval)

            checkbox.stateChanged.connect(lambda state, k=key: self.update_log_setting(k, 'enabled', state == 2))
            combo.currentTextChanged.connect(lambda text, k=key: self.update_log_setting(k, 'interval', int(text)))

            self.log_controls[key] = {'checkbox': checkbox, 'combo': combo}
            grid.addWidget(checkbox, i, 1);
            grid.addWidget(combo, i, 2)

        self.log_on_change_checkbox = QCheckBox("Log core details only on change");
        self.log_on_change_checkbox.setChecked(self.log_on_change)
        self.log_on_change_checkbox.stateChanged.connect(self.on_log_on_change_changed)
        self.start_on_launch_checkbox = QCheckBox("Start logging on launch");
        self.start_on_launch_checkbox.setChecked(self.config.get('start_on_launch', False))
        self.gpu_checkbox = QCheckBox("Measure GPU (NVIDIA)")
        self.gpu_checkbox.setChecked(self.io_sampler.gpu_enabled)
        self.gpu_checkbox.setEnabled(bool(self.io_sampler.nvidia_smi))
        self.gpu_checkbox.stateChanged.connect(lambda state: setattr(self.io_sampler, 'gpu_enabled', state == 2))
        self.tray_checkbox = QCheckBox("Keep running in tray when closed")
        self.tray_checkbox.setChecked(self.config.get('close_to_tray', False))
        grid.addWidget(self.log_on_change_checkbox, len(log_types), 0, 1, 3)
        grid.addWidget(self.start_on_launch_checkbox, len(log_types) + 1, 0, 1, 3)
        grid.addWidget(self.gpu_checkbox, len(log_types) + 2, 0, 1, 3)
        grid.addWidget(self.tray_checkbox, len(log_types) + 3, 0, 1, 3)
        layout.addLayout(grid)

        status_vbox = QVBoxLayout()
        self.status_label = QLabel("STATUS: IDLE")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop)
        status_vbox.addWidget(self.status_label)
        status_vbox.addStretch()
        layout.addLayout(status_vbox)

        return self.create_panel("Control & Logging", layout)

    def on_log_on_change_changed(self, state):
        with self.data_lock:
            self.log_on_change = (state == 2)

    def update_log_setting(self, key, setting_name, value):
        self.log_settings[key][setting_name] = value

    def create_system_status_panel(self):
        layout = QGridLayout()
        self.mem_progress = QProgressBar();
        self.mem_value_label = QLabel("N/A")
        self.swap_progress = QProgressBar();
        self.swap_value_label = QLabel("N/A")
        layout.addWidget(QLabel("Physical Memory:"), 0, 0);
        layout.addWidget(self.mem_progress, 0, 1);
        layout.addWidget(self.mem_value_label, 0, 2)
        layout.addWidget(QLabel("Swap Memory:"), 1, 0);
        layout.addWidget(self.swap_progress, 1, 1);
        layout.addWidget(self.swap_value_label, 1, 2)

        # ディスク・ネットワーク・GPU（Process_Logger-Prototype から移植）
        self.disk_progress = QProgressBar()
        self.disk_value_label = QLabel("N/A")
        self.gpu_progress = QProgressBar()
        self.gpu_value_label = QLabel("N/A")
        self.disk_io_label = QLabel("Disk I/O: N/A")
        self.net_io_label = QLabel("Network: N/A")
        layout.addWidget(QLabel("Disk Usage:"), 2, 0)
        layout.addWidget(self.disk_progress, 2, 1)
        layout.addWidget(self.disk_value_label, 2, 2)
        layout.addWidget(QLabel("GPU Load:"), 3, 0)
        layout.addWidget(self.gpu_progress, 3, 1)
        layout.addWidget(self.gpu_value_label, 3, 2)
        layout.addWidget(self.disk_io_label, 4, 0, 1, 3)
        layout.addWidget(self.net_io_label, 5, 0, 1, 3)
        layout.setColumnStretch(1, 1)
        return self.create_panel("System Status", layout)

    def create_cpu_global_panel(self):
        layout = QHBoxLayout()
        info_layout = QVBoxLayout()

        usage_layout = QHBoxLayout()
        self.avg_cpu_usage_label = QLabel("Avg CPU:")
        self.avg_cpu_usage_bar = QProgressBar()
        usage_layout.addWidget(self.avg_cpu_usage_label);
        usage_layout.addWidget(self.avg_cpu_usage_bar)

        self.package_power_label = QLabel("Power: N/A");
        self.package_temp_label = QLabel("Pkg Temp: N/A")
        self.avg_core_temp_label = QLabel("Avg Temp: N/A");
        self.tjmax_dist_label = QLabel("Margin: N/A")
        self.throttle_label = QLabel("Throttling: N/A")

        grid = QGridLayout()
        grid.addLayout(usage_layout, 0, 0, 1, 2)
        grid.addWidget(self.package_power_label, 1, 0);
        grid.addWidget(self.package_temp_label, 1, 1)
        grid.addWidget(self.avg_core_temp_label, 2, 0);
        grid.addWidget(self.tjmax_dist_label, 2, 1)
        grid.addWidget(self.throttle_label, 3, 0, 1, 2)

        self.temp_graph = TemperatureGraph()
        layout.addLayout(grid, 1);
        layout.addWidget(self.temp_graph, 1)
        return self.create_panel("CPU Global Status", layout)

    def create_core_details_panel(self):
        group = QGroupBox("CPU Core Details")
        layout = QVBoxLayout()
        self.core_table = QTableWidget(self.core_count, 6)
        self.core_table.setHorizontalHeaderLabels(["Core / Thread", "Usage", "Clock", "Temp", "Margin", "Voltage"])
        header_state = self.config.get('table_header_state')
        if header_state:
            self.core_table.horizontalHeader().restoreState(QByteArray.fromHex(header_state.encode()))
        else:
            self.core_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.core_table.verticalHeader().setVisible(False);
        self.core_table.setAlternatingRowColors(True)
        self.core_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        for i, info in enumerate(self.core_info):
            self.core_table.setItem(i, 0, QTableWidgetItem(info['label']))
            self.core_table.setRowHeight(i, 24)
        layout.addWidget(self.core_table);
        group.setLayout(layout)
        return group

    def create_top_processes_panel(self):
        group = QGroupBox("Top Processes")
        layout = QHBoxLayout()

        self.top_cpu_table = QTableWidget(5, 3);
        self.top_cpu_table.setHorizontalHeaderLabels(["Name", "CPU %", "Mem MB"])
        self.top_mem_table = QTableWidget(5, 3);
        self.top_mem_table.setHorizontalHeaderLabels(["Name", "Mem MB", "CPU %"])
        for table in [self.top_cpu_table, self.top_mem_table]:
            table.verticalHeader().setVisible(False)
            for i in range(5): table.setRowHeight(i, 22)

        cpu_box = self.create_panel("By CPU", QVBoxLayout());
        cpu_box.layout().addWidget(self.top_cpu_table)
        mem_box = self.create_panel("By Memory", QVBoxLayout());
        mem_box.layout().addWidget(self.top_mem_table)

        layout.addWidget(cpu_box);
        layout.addWidget(mem_box)
        group.setLayout(layout)
        return group

    def set_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #2E3440; color: #D8DEE9; font-family: 'Segoe UI', sans-serif; }
            QGroupBox { font-size: 14px; border: 1px solid #4C566A; border-radius: 8px; margin-top: 10px; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; }
            QPushButton { background-color: #5E81AC; color: #ECEFF4; border: none; padding: 8px 12px; border-radius: 5px; font-size: 12px; }
            QPushButton:hover { background-color: #81A1C1; } QPushButton:pressed { background-color: #88C0D0; }
            QPushButton:disabled { background-color: #4C566A; color: #6a7388; }
            QLabel { font-size: 12px; } QCheckBox { font-size: 12px; }
            QComboBox { font-size: 12px; background-color: #434C5E; border: 1px solid #4C566A; padding: 4px; border-radius: 4px; }
            QTableWidget { background-color: #3B4252; border: 1px solid #4C566A; gridline-color: #4C566A; font-size: 10px; }
            QHeaderView::section { background-color: #434C5E; color: #ECEFF4; padding: 4px; border: 1px solid #4C566A; font-weight: bold; }
            QProgressBar { border: 1px solid #4C566A; border-radius: 5px; text-align: center; color: #ECEFF4; }
            QProgressBar::chunk { background-color: #88C0D0; border-radius: 4px; }
        """)

    def log_loop(self):
        last_flush_time = time.time();
        FLUSH_INTERVAL = 10
        while self.running:
            current_time = time.time()
            for key, settings in self.log_settings.items():
                if settings['enabled'] and (current_time - settings['last_run'] >= settings['interval']):
                    try:
                        if key in ['detail', 'snapshot']:
                            with self.dll_proc_lock:
                                settings['func'](self.log_paths.get(key))
                        else:
                            settings['func'](self.log_paths.get(key))
                        settings['last_run'] = current_time
                    except Exception as e:
                        print(f"Log Error: {e}")
            if current_time - last_flush_time >= FLUSH_INTERVAL:
                self._flush_core_log_buffer()
                last_flush_time = current_time
            time.sleep(0.5)

    def start_logging(self):
        if self.running: return

        # ★★★ MBCSエンコード（Windows C++ DLL用） ★★★
        encoding_type = 'mbcs'

        self.log_paths = {
            'fast': os.path.join(self.log_session_dir, "fast.csv").encode(encoding_type),
            'detail': os.path.join(self.log_session_dir, "detail.csv").encode(encoding_type),
            'snapshot': os.path.join(self.log_session_dir, "snapshot.csv").encode(encoding_type),
            'corelog': os.path.join(self.log_session_dir, "corelog.csv"),
            'io': os.path.join(self.log_session_dir, "io.csv")
        }
        self.io_logger = None
        if self.log_settings['io']['enabled']:
            self.io_logger = BufferedLogger(self.log_paths['io'], [
                "Timestamp", "Disk_Pct", "Disk_Read_MBps", "Disk_Write_MBps", "Net_Sent_MBps", "Net_Recv_MBps",
                "GPU_Load_Pct", "GPU_Mem_Pct", "GPU_Temp_C"])

        if self.log_settings['fast']['enabled']: dll_pc.pm_log_fast_write_header(self.log_paths['fast'])
        if self.log_settings['detail']['enabled']: dll_pc.pm_detail_active_write_header(self.log_paths['detail'])
        if self.log_settings['snapshot']['enabled']: dll_pc.pm_snapshot_write_header(self.log_paths['snapshot'])
        with self.data_lock:
            self.core_log_buffer.clear();
            self.last_logged_core_data.clear()
        if self.log_settings['corelog']['enabled']:
            try:
                with open(self.log_paths['corelog'], 'w', newline='', encoding='utf-8') as f:
                    f.write(
                        "Timestamp,Throttling_Status,LP_Index,Core_Thread,Usage_Pct,Clock_MHz,Temp_C,Margin_C,Voltage_V,Is_Second_Thread\n")
            except IOError:
                pass
        for key in self.log_settings: self.log_settings[key]['last_run'] = 0
        self.running = True;
        self.status_label.setText("STATUS: 🟢 LOGGING");
        self.start_btn.setEnabled(False);
        self.stop_btn.setEnabled(True)
        for c in self.log_controls.values(): c['checkbox'].setEnabled(False); c['combo'].setEnabled(False)
        self.log_on_change_checkbox.setEnabled(False);
        self.start_on_launch_checkbox.setEnabled(False)
        self.log_thread = threading.Thread(target=self.log_loop, daemon=True);
        self.log_thread.start()
        self._update_tray_tooltip()

    def stop_logging(self):
        if not self.running: return
        self.running = False;
        self.status_label.setText("STATUS: ⏹️ STOPPED");
        self.start_btn.setEnabled(True);
        self.stop_btn.setEnabled(False)
        for c in self.log_controls.values(): c['checkbox'].setEnabled(True); c['combo'].setEnabled(True)
        self.log_on_change_checkbox.setEnabled(True);
        self.start_on_launch_checkbox.setEnabled(True)
        self._flush_core_log_buffer()
        if self.io_logger:
            self.io_logger.force_flush()
        self._update_tray_tooltip()

    def log_io(self, file_path_ignored):
        """ディスク・ネットワーク・GPU の最新値を io.csv に追記する（バッファ経由）。"""
        if not self.io_logger:
            return
        d = self.io_sampler.get()

        def fmt(v, f="{:.3f}"):
            return '' if v is None else f.format(v)

        self.io_logger.add_row([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), fmt(d['disk_pct'], "{:.1f}"),
                                fmt(d['disk_r']), fmt(d['disk_w']), fmt(d['net_sent']), fmt(d['net_recv']),
                                fmt(d['gpu_load'], "{:.0f}"), fmt(d['gpu_mem'], "{:.1f}"),
                                fmt(d['gpu_temp'], "{:.0f}")])

    # ==========================
    # タスクトレイ常駐（Process_Logger-Prototype から移植。PyQt 標準の QSystemTrayIcon を使用）
    # ==========================
    def _setup_tray(self):
        self.tray = None
        if not QSystemTrayIcon.isSystemTrayAvailable():
            return
        icon = self.windowIcon()
        if icon.isNull():
            icon = QIcon(os.path.join(base_dir(), "app.ico"))
        self.tray = QSystemTrayIcon(icon, self)
        menu = QMenu()
        show_action = QAction("Show", self)
        show_action.triggered.connect(self._show_from_tray)
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self._quit_from_tray)
        menu.addAction(show_action)
        menu.addAction(exit_action)
        self.tray.setContextMenu(menu)
        self.tray.activated.connect(
            lambda reason: self._show_from_tray() if reason == QSystemTrayIcon.ActivationReason.DoubleClick else None)
        self._update_tray_tooltip()
        self.tray.show()

    def _update_tray_tooltip(self):
        if getattr(self, 'tray', None):
            self.tray.setToolTip(f"PC Performance Monitor - {'LOGGING' if self.running else 'IDLE'}")

    def _show_from_tray(self):
        self.showNormal()
        self.activateWindow()

    def _quit_from_tray(self):
        self.force_quit = True
        self.close()

    def log_core_details(self, file_path_ignored):
        with self.data_lock:
            if not self.data_cache: return
            data = self.data_cache.copy()
            log_on_change = self.log_on_change

        ts, is_throttling, rows = datetime.now().strftime('%Y-%m-%d %H:%M:%S'), data['cpu_info'].get('is_throttling',
                                                                                                     False), []
        for i in range(self.core_count):
            info = self.core_info[i]
            phys_idx = info['physical_core_num'] - 1
            usage, clock, temp, voltage = data['usage_arr'][i], data['cpu_info']['core_clocks'].get(phys_idx), \
                data['cpu_info']['core_temps'].get(phys_idx), data['cpu_info']['core_voltages'].get(phys_idx)
            margin = (data['cpu_info']['tjmax'] - temp) if temp and data['cpu_info']['tjmax'] else None

            if log_on_change:
                last = self.last_logged_core_data.get(i)
                current = (is_throttling, usage, clock, temp, voltage)
                if last and not self._has_changed(last, current): continue
                self.last_logged_core_data[i] = current
            rows.append(','.join(['' if v is None else str(v) for v in
                                  [ts, is_throttling, i, info['label'], usage, clock, temp, margin, voltage,
                                   info['is_second_thread']]]))
        if rows:
            with self.data_lock: self.core_log_buffer.extend(rows)

    def _has_changed(self, last, current):
        def is_diff(a, b, t): return a is None or b is None or abs(a - b) > t

        return any([last[0] != current[0], is_diff(last[1], current[1], 1.0), is_diff(last[2], current[2], 10.0),
                    is_diff(last[3], current[3], 1.0), is_diff(last[4], current[4], 0.01)])

    def _flush_core_log_buffer(self):
        with self.data_lock:
            if not self.core_log_buffer: return
            lines, self.core_log_buffer = self.core_log_buffer[:], []
        if 'corelog' in self.log_paths and self.log_paths['corelog']:
            try:
                with open(self.log_paths['corelog'], 'a', newline='', encoding='utf-8') as f:
                    f.write('\n'.join(lines) + '\n')
            except IOError:
                pass

    def update_gui(self):
        cpu_info = get_cpu_info()
        usage_arr = (ctypes.c_double * self.core_count)()
        dll_core.get_core_usage(usage_arr, self.core_count)
        with self.data_lock: self.data_cache = {'cpu_info': cpu_info, 'usage_arr': list(usage_arr)}
        self.io_sampler.sample()
        self.update_io_info()
        self.update_memory_info();
        self.update_cpu_global_info(cpu_info, usage_arr);
        self.update_core_table(cpu_info, usage_arr)

    def load_settings(self):
        self.config = {}
        try:
            with open(self.config_file, 'r') as f:
                self.config = json.load(f)
            log_settings = self.config.get('log_settings', {})
            for key, s in log_settings.items():
                if key in self.log_settings: self.log_settings[key].update(s)
            self.log_on_change = self.config.get('log_on_change', True)
        except (FileNotFoundError, json.JSONDecodeError):
            pass

    def save_settings(self):
        config_to_save = {
            'log_settings': {k: {'enabled': s['enabled'], 'interval': s['interval']} for k, s in
                             self.log_settings.items()},
            'log_on_change': self.log_on_change,
            'start_on_launch': self.start_on_launch_checkbox.isChecked(),
            'gpu_enabled': self.gpu_checkbox.isChecked(),
            'close_to_tray': self.tray_checkbox.isChecked(),
            'window_geometry': self.saveGeometry().toHex().data().decode(),
            'table_header_state': self.core_table.horizontalHeader().saveState().toHex().data().decode()
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config_to_save, f, indent=4)
        except IOError:
            pass

    def update_cpu_global_info(self, cpu_info, usage_arr):
        avg_usage = sum(usage_arr) / self.core_count if self.core_count > 0 else 0
        self.avg_cpu_usage_bar.setValue(int(avg_usage));
        self.avg_cpu_usage_bar.setFormat(f"{avg_usage:.1f} %")

        pkg_power = cpu_info.get('package_power')
        self.package_power_label.setText(f"Power: {pkg_power:.1f} W" if pkg_power is not None else "Power: N/A")

        is_throttling = cpu_info.get('is_throttling', False)
        self.throttle_label.setText(f"Throttling: {'YES' if is_throttling else 'NO'}")
        self.throttle_label.setStyleSheet(f"color: {'#BF616A' if is_throttling else '#A3BE8C'}; font-weight: bold;")

        pkg_temp = cpu_info.get('package_temp');
        tjmax = cpu_info.get('tjmax')
        self.package_temp_label.setText(f"Pkg Temp: {pkg_temp:.1f} °C" if pkg_temp is not None else "Pkg Temp: N/A")
        self.tjmax_dist_label.setText(f"Margin: {tjmax - pkg_temp:.1f} °C" if pkg_temp and tjmax else "Margin: N/A")

        core_temps = [t for t in cpu_info.get('core_temps', {}).values() if t]
        avg_temp = sum(core_temps) / len(core_temps) if core_temps else None
        self.avg_core_temp_label.setText(f"Avg Temp: {avg_temp:.1f} °C" if avg_temp is not None else "Avg Temp: N/A")

        if pkg_temp is not None: self.temp_graph.add_point(pkg_temp)

    def update_io_info(self):
        d = self.io_sampler.get()
        if d['disk_pct'] is not None:
            self.disk_progress.setValue(int(d['disk_pct']))
            self.disk_value_label.setText(f"{d['disk_pct']:.1f}% ({_disk_root()})")
        if d['gpu_load'] is not None:
            self.gpu_progress.setValue(int(d['gpu_load']))
            temp = f", {d['gpu_temp']:.0f} °C" if d['gpu_temp'] is not None else ""
            self.gpu_value_label.setText(f"{d['gpu_load']:.0f}%{temp}")
        elif not self.io_sampler.nvidia_smi:
            self.gpu_value_label.setText("N/A (no NVIDIA GPU)")
        self.disk_io_label.setText(f"Disk I/O: R {d['disk_r']:.2f} / W {d['disk_w']:.2f} MB/s")
        self.net_io_label.setText(f"Network: ↑ {d['net_sent']:.2f} / ↓ {d['net_recv']:.2f} MB/s")

    def update_memory_info(self):
        total, used, _, swap_total, swap_used, _ = (ctypes.c_double() for _ in range(6))
        dll_pc.pm_get_memory_totals(ctypes.byref(total), ctypes.byref(used), _, ctypes.byref(swap_total),
                                    ctypes.byref(swap_used), _)
        if total.value > 0:
            pct = int((used.value / total.value) * 100)
            self.mem_progress.setValue(pct);
            self.mem_value_label.setText(f"{used.value:.2f}/{total.value:.2f} GB ({pct}%)")
        if swap_total.value > 0:
            pct = int((swap_used.value / swap_total.value) * 100)
            self.swap_progress.setValue(pct);
            self.swap_value_label.setText(f"{swap_used.value:.2f}/{swap_total.value:.2f} GB ({pct}%)")

    def update_core_table(self, cpu_info, usage_arr):
        temps, clocks, voltages, tjmax = cpu_info.get('core_temps', {}), cpu_info.get('core_clocks', {}), cpu_info.get(
            'core_voltages', {}), cpu_info.get('tjmax')
        for i in range(self.core_count):
            usage_item = self.core_table.item(i, 1)
            new_usage = f"{usage_arr[i]:.1f}"
            if not usage_item or usage_item.text() != new_usage: self.core_table.setItem(i, 1,
                                                                                         QTableWidgetItem(new_usage))
            if self.core_info[i]['is_second_thread']:
                if not self.core_table.item(i, 2) or self.core_table.item(i, 2).text() != "Same as above":
                    for col in [2, 3, 4, 5]: self.core_table.setItem(i, col, QTableWidgetItem("Same as above"))
            else:
                phys_idx = self.core_info[i]['physical_core_num'] - 1
                vals = [clocks.get(phys_idx), temps.get(phys_idx),
                        (tjmax - temps.get(phys_idx)) if tjmax and temps.get(phys_idx) else None,
                        voltages.get(phys_idx)]
                fmts = ["{:.0f}", "{:.1f}", "{:.1f}", "{:.3f}"]
                for col, (val, fmt) in enumerate(zip(vals, fmts), 2):
                    item = self.core_table.item(i, col)
                    new_text = fmt.format(val) if val is not None else "N/A"
                    if not item or item.text() != new_text: self.core_table.setItem(i, col, QTableWidgetItem(new_text))

    def update_top_processes(self):
        """DLLを使用してトッププロセス情報を取得し、テーブルを更新する"""
        try:
            with self.dll_proc_lock:
                topN = 5
                top_cpu_procs, top_mem_procs = (PROCINFO * topN)(), (PROCINFO * topN)()
                num_found = dll_pc.pm_get_top_processes(topN, top_cpu_procs, top_mem_procs)

            for i in range(topN):
                if i < num_found:
                    cpu_p, mem_p = top_cpu_procs[i], top_mem_procs[i]
                    cpu_pct_c = cpu_p.cpu / self.core_count if self.core_count > 0 else cpu_p.cpu
                    mem_mb_c = cpu_p.ram / (1024 * 1024)
                    self.top_cpu_table.setItem(i, 0, QTableWidgetItem(cpu_p.name.decode('utf-8', 'replace')))
                    self.top_cpu_table.setItem(i, 1, QTableWidgetItem(f"{cpu_pct_c:.1f}"))
                    self.top_cpu_table.setItem(i, 2, QTableWidgetItem(f"{mem_mb_c:.1f}"))

                    cpu_pct_m = mem_p.cpu / self.core_count if self.core_count > 0 else mem_p.cpu
                    mem_mb_m = mem_p.ram / (1024 * 1024)
                    self.top_mem_table.setItem(i, 0, QTableWidgetItem(mem_p.name.decode('utf-8', 'replace')))
                    self.top_mem_table.setItem(i, 1, QTableWidgetItem(f"{mem_mb_m:.1f}"))
                    self.top_mem_table.setItem(i, 2, QTableWidgetItem(f"{cpu_pct_m:.1f}"))
                else:
                    for j in range(3): self.top_cpu_table.setItem(i, j,
                                                                  QTableWidgetItem("")); self.top_mem_table.setItem(i,
                                                                                                                    j,
                                                                                                                    QTableWidgetItem(
                                                                                                                        ""))

            for table in [self.top_cpu_table, self.top_mem_table]:
                table.resizeColumnsToContents()
                table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        except Exception as e:
            print(f"Top Process Error: {e}")

    def closeEvent(self, event):
        # 「トレイに残す」設定なら、×ボタンでは隠すだけにして記録を続ける
        if self.tray and self.tray_checkbox.isChecked() and not self.force_quit:
            event.ignore()
            self.hide()
            self.save_settings()
            self.tray.showMessage("PC Performance Monitor", "Running in the tray. Right-click the icon to exit.",
                                  QSystemTrayIcon.MessageIcon.Information, 3000)
            return
        self.running = False
        self._flush_core_log_buffer();
        if self.io_logger:
            self.io_logger.force_flush()
        self.save_settings()
        if self.tray:
            self.tray.hide()
        if computer: computer.Close()
        super().closeEvent(event)


if __name__ == "__main__":
    import clr

    if DLLS_DIR not in sys.path:
        sys.path.append(DLLS_DIR)

    try:
        clr.AddReference("LibreHardwareMonitorLib")
    except Exception:
        sys.exit(1)

    app = QApplication(sys.argv)
    initialize_hardware_monitor()
    win = MonitorApp()
    win.show()
    sys.exit(app.exec())