import sys
import os
import math
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

        self.prev_active = False
        self.prev_jogging = False
        self.prev_mode = MODE_IDLE
        self.first_sync_done = False
        # --- CẤU HÌNH VỊ TRÍ HOME ---
        self.HOME_POS = [0.161, 0.000, 0.120] # X, Y, Z
        self.HOME_RPY = [0.108, 3.14, -3.14] # R, P, Y
        
        # --- TRẠNG THÁI NỘI BỘ ---
        self.is_active = False
        self.current_mode = MODE_POSE
        self.target_pos = [0.0]*3
        self.target_rpy = [0.0]*3
        self.traj_duration = 4.0
        self.traj_trigger = 0
        self.manual_vel = [0.0]*6
        self.cmd_gripper = 0.0 

        self.is_stuck = False
        self.current_error = 0.0
        self.last_fb_pose = [0.0] * 6 # Lưu Feedback: [X, Y, Z, R, P, Y]

        self.status_label = None 
        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(30)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        master_layout = QVBoxLayout(central)
        content_layout = QHBoxLayout()
        
        # 1. Panel bên trái: Feedback
        self.monitor = MonitorPanel()
        content_layout.addWidget(self.monitor, 4)
        
        # 2. Panel bên phải: Control
        right_panel = QVBoxLayout()
        
        top_btns_layout = QHBoxLayout()
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(50)
        self.btn_active.clicked.connect(self.toggle_active)
        
        self.btn_home = QPushButton("GO HOME")
        self.btn_home.setMinimumHeight(50)
        self.btn_home.setStyleSheet("background-color: #0277BD; color: white; font-weight: bold;")
        self.btn_home.clicked.connect(self.go_home)
        
        top_btns_layout.addWidget(self.btn_active, 7)
        top_btns_layout.addWidget(self.btn_home, 3)
        right_panel.addLayout(top_btns_layout)

        # Gripper Control
        grip_gb = QGroupBox("Gripper Control")
        grip_layout = QHBoxLayout()
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 100) 
        self.grip_slider.setValue(0)
        self.grip_slider.setEnabled(False) 
        self.grip_slider.valueChanged.connect(self.on_gripper_change)
        grip_layout.addWidget(QLabel("Open"))
        grip_layout.addWidget(self.grip_slider)
        grip_layout.addWidget(QLabel("Close"))
        grip_gb.setLayout(grip_layout)
        right_panel.addWidget(grip_gb)

        # Tabs điều khiển
        self.tabs = ControlTabs(self)
        right_panel.addWidget(self.tabs)
        content_layout.addLayout(right_panel, 3)
        
        master_layout.addLayout(content_layout)

        self.status_label = QLabel("SYSTEM READY")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setMinimumHeight(40)
        self.status_label.setStyleSheet("background-color: #212121; color: white; font-weight: bold;")
        master_layout.addWidget(self.status_label)

    def on_gripper_change(self, val):
        self.cmd_gripper = val / 100.0

    def toggle_active(self):
        self.is_active = self.btn_active.isChecked()
        self.grip_slider.setEnabled(self.is_active)
        if self.is_active:
            self.btn_active.setText("SYSTEM ACTIVE")
            self.btn_active.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
        else:
            self.btn_active.setText("ENABLE CONTROL")
            self.btn_active.setStyleSheet("background-color: #546E7A; color: white;")

    def go_home(self):
        if not self.is_active: return
        self.target_pos = list(self.HOME_POS)
        self.target_rpy = list(self.HOME_RPY)
        self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)
        self.current_mode = MODE_TRAJECTORY
        self.traj_duration = 3.0
        self.traj_trigger += 1
        self.status_label.setText("STATUS: RETURNING HOME...")

    def loop(self):
        # 1. KẾT NỐI & ĐỌC FEEDBACK
        if not self.shm.shm:
            if not self.shm.connect(): return
        fb = self.shm.read_feedback()
        if not fb: return

        # Cập nhật hiển thị và lưu tọa độ thực tế
        self.monitor.update_display(fb)
        self.last_fb_pose = list(fb['pose'])

        # 2. XÁC ĐỊNH CHẾ ĐỘ DỰA TRÊN TAB HIỆN TẠI (Source of Truth)
        # Giả sử: Tab 0 = Pose, Tab 1 = Trajectory, Tab 2 = Joint
        tab_index = self.tabs.currentIndex()
        
        if not self.is_active:
            new_mode = MODE_IDLE
        elif tab_index == 0:
            new_mode = MODE_POSE
        elif tab_index == 1:
            new_mode = MODE_TRAJECTORY
        elif tab_index == 2:
            new_mode = MODE_JOINT
        else:
            new_mode = MODE_IDLE

        # 3. LẤY DỮ LIỆU TƯƠNG ỨNG VỚI MODE ĐANG CHỌN
        # Reset vận tốc manual về 0 trước khi lấy dữ liệu mới
        current_manual_vel = [0.0] * 6
        
        if new_mode == MODE_JOINT:
            # Chỉ lấy vận tốc khớp khi đang ở Tab Joint
            current_manual_vel = self.tabs.joint_tab.get_velocities()
        elif new_mode == MODE_TRAJECTORY:
            # Nếu có nút Jogging bên Tab Trajectory/Pose thì lấy ở đó (tùy GUI của bạn)
            # Ở đây tôi ví dụ lấy manual_vel từ thuộc tính chung
            current_manual_vel = self.manual_vel 

        # 4. LOGIC ĐỒNG BỘ (SYNC)
        # Đồng bộ khi: Mới mở máy, Chuyển từ Joint sang Pose, Chuyển từ Traj sang Pose
        need_sync = False
        if not self.first_sync_done: 
            need_sync = True
            self.first_sync_done = True

        if self.is_active and not self.prev_active: 
            need_sync = True

        # Khi đang ở mode Joint hoặc vừa thoát khỏi Trajectory/Joint để về Pose
        if new_mode == MODE_JOINT or (new_mode == MODE_POSE and self.prev_mode != MODE_POSE):
            need_sync = True

        if need_sync:
            self.target_pos = self.last_fb_pose[:3]
            self.target_rpy = self.last_fb_pose[3:]
            self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)

        # Cập nhật các biến trạng thái chính
        self.current_mode = new_mode
        self.manual_vel = current_manual_vel # Cập nhật mảng vận tốc cuối cùng
        
        self.current_error = math.sqrt(sum((self.target_pos[i] - self.last_fb_pose[i])**2 for i in range(3)))

        # 5. XỬ LÝ ĐẶC BIỆT CHO MODE TRAJECTORY (Sequence)
        if self.current_mode == MODE_TRAJECTORY and self.seq_manager.is_running:
            mgr = self.seq_manager
            step = mgr.steps[mgr.current_idx]
            self.target_pos = step['pos']
            self.target_rpy = step['rpy']
            self.cmd_gripper = step['grip']
            self.traj_duration = step['dur']
            self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)
            
            mgr.timer_count += 0.03
            if mgr.timer_count >= self.traj_duration:
                mgr.timer_count = 0; mgr.current_idx += 1; self.traj_trigger += 1
                if mgr.current_idx >= len(mgr.steps):
                    mgr.is_running = False

        # 6. GHI XUỐNG SHARED MEMORY (Đúng Offset)
        self.shm.write_command(
            self.is_active, 
            self.current_mode, 
            self.target_pos, 
            self.target_rpy, 
            self.traj_duration, 
            self.traj_trigger, 
            self.manual_vel, 
            self.cmd_gripper
        )

        # 7. LƯU TRẠNG THÁI
        self.prev_active = self.is_active
        self.prev_mode = self.current_mode
        self.refresh_status_ui()

    def refresh_status_ui(self):
        """Hàm phụ trợ cập nhật màu sắc và thông báo trạng thái"""
        if not self.is_active:
            self.status_label.setText("IDLE - STANDBY")
            self.status_label.setStyleSheet("background-color: #212121; color: white;")
            return

        if self.current_error > MAX_ALLOWED_ERROR:
            self.status_label.setText(f"⚠️ LIMIT REACHED! Error: {self.current_error:.3f}m")
            self.status_label.setStyleSheet("background-color: #D32F2F; color: white; font-weight: bold;")
        else:
            mode_map = {MODE_POSE: "POSE", MODE_TRAJECTORY: "TRAJECTORY", MODE_JOINT: "JOINT"}
            mode_str = mode_map.get(self.current_mode, "UNKNOWN")
            self.status_label.setText(f"SYSTEM ACTIVE - MODE: {mode_str}")
            self.status_label.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RobotGUI(); win.show()
    sys.exit(app.exec())