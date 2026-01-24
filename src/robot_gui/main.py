import sys
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, 
                             QVBoxLayout, QPushButton, QSlider, QLabel, QGroupBox)
from PyQt6.QtCore import QTimer, Qt

# Đảm bảo Python tìm thấy các module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from shm_manager import SHMManager
from sequence_manager import SequenceManager
from tabs.monitor_panel import MonitorPanel
from tabs.control_tabs import ControlTabs
from config import *

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Modular Interface - Full Control")
        self.resize(1200, 850)
        
        # --- QUẢN LÝ DỮ LIỆU ---
        self.shm = SHMManager()
        self.seq_manager = SequenceManager()
        
        # --- TRẠNG THÁI NỘI BỘ ---
        self.is_active = False
        self.current_mode = MODE_POSE
        self.target_pos = [0.0]*3
        self.target_rpy = [0.0]*3
        self.traj_duration = 2.0
        self.traj_trigger = 0
        self.manual_vel = [0.0]*6
        self.cmd_gripper = 0.0 # <--- Biến lưu giá trị kẹp

        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(30)

    def init_ui(self):
        central = QWidget()
        layout = QHBoxLayout(central)
        
        # 1. Panel bên trái: Feedback
        self.monitor = MonitorPanel()
        layout.addWidget(self.monitor, 4)
        
        # 2. Panel bên phải: Control
        right_panel = QVBoxLayout()
        
        # Nút Active
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(50)
        self.btn_active.clicked.connect(self.toggle_active)
        right_panel.addWidget(self.btn_active)

        # --- PHẦN ĐIỀU KHIỂN KẸP (GRIPPER) ---
        grip_gb = QGroupBox("Gripper Control")
        grip_layout = QHBoxLayout()
        
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 100) # 0: Mở hoàn toàn, 100: Đóng hoàn toàn
        self.grip_slider.setValue(0)
        self.grip_slider.setEnabled(False) # Khóa khi chưa active
        self.grip_slider.valueChanged.connect(self.on_gripper_change)
        
        grip_layout.addWidget(QLabel("Open"))
        grip_layout.addWidget(self.grip_slider)
        grip_layout.addWidget(QLabel("Close"))
        grip_gb.setLayout(grip_layout)
        right_panel.addWidget(grip_gb)
        # ------------------------------------

        # Tabs điều khiển
        self.tabs = ControlTabs(self)
        right_panel.addWidget(self.tabs)
        
        layout.addLayout(right_panel, 3)
        self.setCentralWidget(central)

    def on_gripper_change(self, val):
        """Chuyển giá trị slider 0-100 thành 0.0-1.0 gửi xuống C++"""
        self.cmd_gripper = -val / 100.0

    def toggle_active(self):
        self.is_active = self.btn_active.isChecked()
        # Bật/Tắt thanh trượt kẹp theo trạng thái hệ thống
        self.grip_slider.setEnabled(self.is_active)
        
        if self.is_active:
            self.btn_active.setText("SYSTEM ACTIVE")
            self.btn_active.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        else:
            self.btn_active.setText("ENABLE CONTROL")
            self.btn_active.setStyleSheet("background-color: #546E7A; color: white;")

    def loop(self):
        if not self.shm.shm:
            self.shm.connect()
            return

        fb = self.shm.read_feedback()
        if fb:
            self.monitor.update_display(fb)
            if not self.is_active:
                self.target_pos = list(fb['pose'][:3])
                self.target_rpy = list(fb['pose'][3:])
                self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)

        # Sequence Logic
        mgr = self.seq_manager
        if self.is_active and mgr.is_running:
            step = mgr.steps[mgr.current_idx]
            self.target_pos, self.target_rpy = step['pos'], step['rpy']
            self.cmd_gripper = step['grip']
            self.traj_duration = step['dur']
            
            # Cập nhật thanh trượt kẹp trên giao diện khi chạy tự động
            self.grip_slider.blockSignals(True)
            self.grip_slider.setValue(int(self.cmd_gripper * 100))
            self.grip_slider.blockSignals(False)
            
            self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)
            
            mgr.timer_count += 0.03
            if mgr.timer_count >= self.traj_duration:
                mgr.timer_count = 0
                mgr.current_idx += 1
                self.traj_trigger += 1
                if mgr.current_idx >= len(mgr.steps):
                    mgr.is_running = False
                    self.tabs.seq_tab.btn_run.setText("RUN SEQUENCE")

        # GỬI DỮ LIỆU XUỐNG C++ (Bao gồm cả self.cmd_gripper)
        self.shm.write_command(
            self.is_active, self.current_mode, self.target_pos, 
            self.target_rpy, self.traj_duration, self.traj_trigger, 
            self.manual_vel, self.cmd_gripper
        )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = RobotGUI()
    win.show()
    sys.exit(app.exec())