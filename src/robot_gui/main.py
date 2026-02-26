import sys
import os
import math
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, 
                             QVBoxLayout, QPushButton, QSlider, QLabel, QGroupBox)
from PyQt6.QtCore import QTimer, Qt

# Đảm bảo Python tìm thấy các module trong thư mục hiện tại
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from shm_manager import SHMManager
from sequence_manager import SequenceManager
from tabs.monitor_panel import MonitorPanel
from tabs.control_tabs import ControlTabs
from config import *

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Modular Interface - Professional Control")
        self.resize(1300, 850)
        
        # --- QUẢN LÝ DỮ LIỆU & MODULE ---
        self.shm = SHMManager()
        self.seq_manager = SequenceManager()
        
        # --- CẤU HÌNH VỊ TRÍ ---
        self.HOME_POS = [0.161, 0.000, 0.120]
        self.HOME_RPY = [0.108, 3.14, -3.14]
        
        # --- TRẠNG THÁI ĐIỀU KHIỂN (GỬI XUỐNG ROBOT) ---
        self.is_active = False
        self.current_mode = MODE_IDLE
        self.target_pos = [0.0] * 3
        self.target_rpy = [0.0] * 3
        self.ee_vel = [0.0] * 6     # [vx, vy, vz, wx, wy, wz]
        self.manual_vel = [0.0] * 6 # [j1, j2, j3, j4, j5, j6]
        self.traj_duration = 3.0
        self.traj_trigger = 0
        self.cmd_gripper = 0.0 

        # --- TRẠNG THÁI PHẢN HỒI & LOGIC ---
        self.last_fb_pose = [0.0] * 6
        self.last_is_active = False
        self.last_jogging_state = False 
        self.is_initialized = False # Cờ để thực hiện đồng bộ vị trí ngay khi bật app

        self.init_ui()
        
        # --- TIMER VÒNG LẶP CHÍNH (30ms) ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(30)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        master_layout = QVBoxLayout(central)
        content_layout = QHBoxLayout()
        
        # Monitor Panel (Bên trái)
        self.monitor = MonitorPanel()
        content_layout.addWidget(self.monitor, 4)
        
        # Control Panel (Bên phải)
        right_panel = QVBoxLayout()
        
        top_btns = QHBoxLayout()
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(55)
        self.btn_active.clicked.connect(self.toggle_active)
        
        self.btn_home = QPushButton("GO HOME")
        self.btn_home.setMinimumHeight(55)
        self.btn_home.setStyleSheet("background-color: #0277BD; color: white; font-weight: bold;")
        self.btn_home.clicked.connect(self.go_home)
        
        top_btns.addWidget(self.btn_active, 7)
        top_btns.addWidget(self.btn_home, 3)
        right_panel.addLayout(top_btns)

        # Gripper
        grip_gb = QGroupBox("Gripper Control")
        grip_l = QHBoxLayout()
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 100) 
        self.grip_slider.setValue(0)
        self.grip_slider.setEnabled(False) 
        self.grip_slider.valueChanged.connect(self.on_gripper_change)
        grip_l.addWidget(QLabel("Open"))
        grip_l.addWidget(self.grip_slider)
        grip_l.addWidget(QLabel("Close"))
        grip_gb.setLayout(grip_l)
        right_panel.addWidget(grip_gb)

        # Tabs
        self.tabs = ControlTabs(self)
        right_panel.addWidget(self.tabs)
        content_layout.addLayout(right_panel, 3)
        
        master_layout.addLayout(content_layout)

        # Status Label
        self.status_label = QLabel("SYSTEM STANDBY")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setMinimumHeight(45)
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
        self.traj_trigger += 1

    def loop(self):
        if not self.shm.shm:
            if not self.shm.connect(): return

        fb = self.shm.read_feedback()
        if fb:
            self.monitor.update_display(fb)
            self.last_fb_pose = list(fb['pose']) 
            
            # 1. FIX KHỞI TẠO: Đồng bộ vị trí Robot thực vào GUI ngay khi vừa mở app
            if not self.is_initialized:
                self.target_pos = self.last_fb_pose[:3]
                self.target_rpy = self.last_fb_pose[3:]
                self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)
                self.is_initialized = True

            # 2. LẤY VẬN TỐC EE VÀ VỊ TRÍ MỤC TIÊU TỪ TAB
            current_ee_vel = self.tabs.vel_tab.get_current_vel()  # Feedforward only
            vel_target_pos = self.tabs.vel_tab.get_target_pos()   # Vị trí mục tiêu toán học

            # 3. KIỂM TRA TRẠNG THÁI JOGGING
            is_manual_jogging = any(abs(v) > 1e-6 for v in self.manual_vel)
            is_vel_test = vel_target_pos is not None  # Circle/Line test đang chạy
            is_jogging = is_manual_jogging or is_vel_test

            # 4. QUẢN LÝ TARGET POSITION
            if is_vel_test:
                # Circle/Line test: Ghi vị trí mục tiêu toán học vào target_pos
                # Controller C++ sẽ dùng target_pos này để feedback tại 500Hz
                self.target_pos = list(vel_target_pos)
                self.target_rpy = self.last_fb_pose[3:]  # Giữ orientation theo actual
            elif is_manual_jogging:
                # Manual jog: Sync target = actual để khi nhả nút robot đứng yên
                self.target_pos = self.last_fb_pose[:3]
                self.target_rpy = self.last_fb_pose[3:]
                self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)

            # Đồng bộ UI khi Standby hoặc vừa thả nút Jogging
            elif not self.is_active or (not is_jogging and self.last_jogging_state):
                self.tabs.pose_tab.update_ui_values(self.last_fb_pose[:3], self.last_fb_pose[3:])
            
            self.last_jogging_state = is_jogging
            self.last_is_active = self.is_active

            # 5. QUẢN LÝ MODE THEO TAB
            if self.is_active:
                tab_idx = self.tabs.currentIndex() # 0: Cartesian, 1: Seq, 2: Joint, 3: Velocity

                if self.seq_manager.is_running:
                    # Sequence Mode
                    step = self.seq_manager.steps[self.seq_manager.current_idx]
                    self.target_pos, self.target_rpy = step['pos'], step['rpy']
                    self.cmd_gripper, self.traj_duration = step['grip'], step['dur']
                    self.tabs.pose_tab.update_ui_values(self.target_pos, self.target_rpy)
                    self.current_mode = MODE_TRAJECTORY
                    
                    self.seq_manager.timer_count += 0.03
                    if self.seq_manager.timer_count >= self.traj_duration:
                        self.seq_manager.timer_count = 0
                        self.seq_manager.current_idx += 1
                        self.traj_trigger += 1
                        if self.seq_manager.current_idx >= len(self.seq_manager.steps):
                            self.seq_manager.is_running = False
                            self.tabs.seq_tab.btn_run.setText("RUN SEQUENCE")
                
                elif any(abs(v) > 1e-6 for v in self.manual_vel):
                    # Ưu tiên Mode Joint nếu đang nhấn nút ở Tab Joint
                    self.current_mode = MODE_JOINT
                
                elif any(abs(v) > 1e-6 for v in current_ee_vel):
                    # Ưu tiên Mode Velocity nếu đang nhấn nút ở Tab Velocity
                    self.current_mode = MODE_TRAJECTORY
                    self.ee_vel = current_ee_vel
                
                elif tab_idx == 0:
                    # Chế độ Cartesian: Chỉ cập nhật target từ SpinBox khi người dùng nhấn nút GO (xử lý trong PoseTab)
                    # Hoặc nếu muốn robot luôn bám SpinBox thì mở dòng dưới:
                    # self.target_pos = self.tabs.pose_tab.get_pos()
                    self.current_mode = MODE_POSE
                
                else:
                    self.current_mode = MODE_POSE

            else:
                self.current_mode = MODE_IDLE

            # 5.5. CẬP NHẬT BIỂU ĐỒ QUỸ ĐẠO
            # Lấy vị trí mục tiêu từ VelocityTab (circle/line test) để vẽ so sánh
            target_plot = self.tabs.vel_tab.get_target_pos()
            self.tabs.plot_tab.update_plot(self.last_fb_pose[:3], target_pos=target_plot)

            # 6. STATUS UI
            err = math.sqrt(sum((self.target_pos[i] - self.last_fb_pose[i])**2 for i in range(3)))
            if self.is_active:
                if err > MAX_ALLOWED_ERROR:
                    self.status_label.setText(f"⚠️ LIMIT ERROR: {err:.3f}m")
                    self.status_label.setStyleSheet("background-color: #D32F2F; color: white;")
                else:
                    self.status_label.setText(f"ACTIVE - MODE: {self.current_mode}")
                    self.status_label.setStyleSheet("background-color: #2E7D32; color: white;")
            else:
                self.status_label.setText("IDLE - STANDBY")
                self.status_label.setStyleSheet("background-color: #212121; color: white;")

            # 7. GHI XUỐNG SHARED MEMORY
            # Tách ee_vel thành lin và ang nếu shm_manager yêu cầu, hoặc gửi cả cục 6
            self.shm.write_command(
                active=self.is_active, 
                mode=self.current_mode, 
                target_p=self.target_pos, 
                target_r=self.target_rpy, 
                dur=self.traj_duration, 
                trig=self.traj_trigger, 
                manual_j=self.manual_vel, 
                gripper=self.cmd_gripper,
                ee_vel=self.ee_vel if self.current_mode == MODE_TRAJECTORY else [0.0]*6
            )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RobotGUI()
    win.show()
    sys.exit(app.exec())