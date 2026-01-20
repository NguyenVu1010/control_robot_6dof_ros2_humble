import sys
import mmap
import struct
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, QSlider, QPushButton, 
                             QGridLayout, QProgressBar, QMessageBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor, QPalette

# --- CẤU HÌNH SHARED MEMORY ---
# Phải khớp với C++ struct RobotData
# offsets:
# 0-48:   Joint Pos (6 double)
# 48-96:  Joint Vel (6 double)
# 96-120: EE Pos (3 double)
# 120-144: EE RPY (3 double)
# 144-168: Cmd Linear (3 double)
# 168-192: Cmd Angular (3 double)
# 192:     Cmd Active (bool)
# 193:     Sys Ready (bool)
SHM_NAME = "/robot_control_shm"
SHM_SIZE = 200 # Ước lượng an toàn

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface (Shared Memory)")
        self.setGeometry(100, 100, 900, 600)
        self.shm = None
        
        # Biến lưu lệnh điều khiển
        self.cmd_lin = [0.0, 0.0, 0.0]
        self.cmd_ang = [0.0, 0.0, 0.0]
        self.is_active = False

        self.init_ui()
        self.connect_shm()

        # Timer cập nhật giao diện (30 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33) 

    def connect_shm(self):
        try:
            # Mở shared memory file trong /dev/shm
            f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(f.fileno(), 0)
            self.status_label.setText("STATUS: CONNECTED")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        except FileNotFoundError:
            self.status_label.setText("STATUS: ROBOT NOT RUNNING")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            QMessageBox.critical(self, "Error", "Không tìm thấy Robot Controller!\nHãy chạy 'ros2 launch...' trước.")

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # --- CỘT TRÁI: HIỂN THỊ THÔNG TIN ---
        left_panel = QVBoxLayout()
        
        # 1. Joint States Group
        gb_joints = QGroupBox("Joint Positions (rad)")
        layout_joints = QGridLayout()
        self.joint_bars = []
        self.joint_labels = []
        for i in range(6):
            lbl_name = QLabel(f"J{i+1}:")
            val_lbl = QLabel("0.00")
            pbar = QProgressBar()
            pbar.setRange(-314, 314) # -3.14 đến 3.14 rad
            pbar.setTextVisible(False)
            
            layout_joints.addWidget(lbl_name, i, 0)
            layout_joints.addWidget(pbar, i, 1)
            layout_joints.addWidget(val_lbl, i, 2)
            
            self.joint_bars.append(pbar)
            self.joint_labels.append(val_lbl)
        gb_joints.setLayout(layout_joints)
        left_panel.addWidget(gb_joints)

        # 2. End-Effector Group
        gb_ee = QGroupBox("End-Effector Pose")
        layout_ee = QGridLayout()
        self.ee_labels = {}
        tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for i, tag in enumerate(tags):
            lbl = QLabel(f"{tag}:")
            val = QLabel("0.000")
            val.setFont(QFont("Arial", 12, QFont.Weight.Bold))
            layout_ee.addWidget(lbl, i, 0)
            layout_ee.addWidget(val, i, 1)
            self.ee_labels[tag] = val
        gb_ee.setLayout(layout_ee)
        left_panel.addWidget(gb_ee)

        # --- CỘT PHẢI: ĐIỀU KHIỂN ---
        right_panel = QVBoxLayout()

        # 1. Control Switch
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setStyleSheet("background-color: gray; color: white; padding: 10px;")
        self.btn_active.clicked.connect(self.toggle_control)
        right_panel.addWidget(self.btn_active)

        # 2. Velocity Sliders
        gb_cmd = QGroupBox("Velocity Command")
        layout_cmd = QVBoxLayout()
        
        self.sliders = {}
        axes = [("Linear X", 0, 0), ("Linear Y", 0, 1), ("Linear Z", 0, 2),
                ("Angular R", 1, 0), ("Angular P", 1, 1), ("Angular Y", 1, 2)]
        
        for name, type_idx, axis_idx in axes:
            h_layout = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setFixedWidth(80)
            
            # Slider range: -100 to 100 (tương ứng -0.5 m/s đến 0.5 m/s)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-100, 100)
            slider.setValue(0)
            slider.setEnabled(False)
            slider.valueChanged.connect(lambda val, t=type_idx, a=axis_idx: self.update_cmd(t, a, val))
            
            val_lbl = QLabel("0.00")
            val_lbl.setFixedWidth(40)
            
            h_layout.addWidget(lbl)
            h_layout.addWidget(slider)
            h_layout.addWidget(val_lbl)
            layout_cmd.addLayout(h_layout)
            
            self.sliders[f"{type_idx}_{axis_idx}"] = (slider, val_lbl)

        # Nút Stop khẩn cấp
        btn_stop = QPushButton("EMERGENCY STOP")
        btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;")
        btn_stop.clicked.connect(self.stop_robot)
        layout_cmd.addWidget(btn_stop)
        
        gb_cmd.setLayout(layout_cmd)
        right_panel.addWidget(gb_cmd)

        # Status Footer
        self.status_label = QLabel("STATUS: DISCONNECTED")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Add to main layout
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 1)
        
        layout_container = QVBoxLayout()
        layout_container.addLayout(main_layout)
        layout_container.addWidget(self.status_label)
        
        main_widget.setLayout(layout_container)
        self.setCentralWidget(main_widget)

    def toggle_control(self):
        self.is_active = self.btn_active.isChecked()
        if self.is_active:
            self.btn_active.setText("CONTROL ACTIVE")
            self.btn_active.setStyleSheet("background-color: green; color: white; padding: 10px;")
            # Enable Sliders
            for s, l in self.sliders.values(): s.setEnabled(True)
        else:
            self.btn_active.setText("ENABLE CONTROL")
            self.btn_active.setStyleSheet("background-color: gray; color: white; padding: 10px;")
            self.stop_robot() # Reset về 0
            # Disable Sliders
            for s, l in self.sliders.values(): s.setEnabled(False)

    def update_cmd(self, type_idx, axis_idx, val):
        # Map -100..100 -> -0.5..0.5
        real_val = val / 200.0 
        if type_idx == 0: self.cmd_lin[axis_idx] = real_val
        else: self.cmd_ang[axis_idx] = real_val
        
        # Update Label
        slider, lbl = self.sliders[f"{type_idx}_{axis_idx}"]
        lbl.setText(f"{real_val:.2f}")

    def stop_robot(self):
        self.cmd_lin = [0.0, 0.0, 0.0]
        self.cmd_ang = [0.0, 0.0, 0.0]
        for s, l in self.sliders.values():
            s.blockSignals(True)
            s.setValue(0)
            l.setText("0.00")
            s.blockSignals(False)

    def update_loop(self):
        if self.shm is None: return

        # --- 1. ĐỌC DỮ LIỆU TỪ C++ ---
        try:
            self.shm.seek(0)
            # Đọc 18 double (Joint Pos + Joint Vel + EE Pos + EE RPY)
            # 6 + 6 + 3 + 3 = 18 doubles = 144 bytes
            data_bytes = self.shm.read(144)
            data = struct.unpack('18d', data_bytes)
            
            # Cập nhật GUI
            # Joints (0-5)
            for i in range(6):
                val = data[i]
                self.joint_bars[i].setValue(int(val * 100))
                self.joint_labels[i].setText(f"{val:.3f}")
            
            # EE Pos (12-14)
            self.ee_labels["X"].setText(f"{data[12]:.4f}")
            self.ee_labels["Y"].setText(f"{data[13]:.4f}")
            self.ee_labels["Z"].setText(f"{data[14]:.4f}")
            
            # EE RPY (15-17)
            self.ee_labels["Roll"].setText(f"{data[15]:.3f}")
            self.ee_labels["Pitch"].setText(f"{data[16]:.3f}")
            self.ee_labels["Yaw"].setText(f"{data[17]:.3f}")

        except Exception as e:
            print(f"Read Error: {e}")

        # --- 2. GHI LỆNH XUỐNG C++ ---
        try:
            # Ghi lệnh vào offset 144
            # 3 double lin + 3 double ang = 6 doubles
            cmd_bytes = struct.pack('6d', 
                                    self.cmd_lin[0], self.cmd_lin[1], self.cmd_lin[2],
                                    self.cmd_ang[0], self.cmd_ang[1], self.cmd_ang[2])
            self.shm.seek(144)
            self.shm.write(cmd_bytes)
            
            # Ghi cờ Active vào offset 192 (1 byte bool)
            self.shm.seek(192)
            self.shm.write(struct.pack('?', self.is_active))
            
        except Exception as e:
            print(f"Write Error: {e}")

    def closeEvent(self, event):
        # Khi tắt app, gửi lệnh dừng và ngắt kết nối
        if self.shm:
            self.stop_robot()
            self.shm.seek(192)
            self.shm.write(struct.pack('?', False)) # Active = False
            self.shm.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Set dark theme for cool look
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(palette)

    window = RobotInterface()
    window.show()
    sys.exit(app.exec())