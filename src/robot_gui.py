import sys
import mmap
import struct
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, QSlider, QPushButton, 
                             QGridLayout, QProgressBar, QMessageBox, QDoubleSpinBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor, QPalette

# --- CẤU HÌNH SHARED MEMORY ---
# Phải khớp 100% với struct RobotData bên C++
# 0-48:    Joint Pos (6 double)
# 48-96:   Joint Vel (6 double)
# 96-120:  EE Pos (3 double)
# 120-144: EE RPY (3 double)
# 144-168: Target Pos (3 double)
# 168-192: Target RPY (3 double)
# 192-200: Cmd Gripper (1 double)
# 200:     Cmd Active (bool)
# 201:     Sys Ready (bool)
SHM_NAME = "/robot_control_shm"

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface (Position Mode)")
        self.setGeometry(100, 100, 1100, 700)
        self.shm = None
        
        # Biến lưu trữ lệnh
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_rpy = [0.0, 0.0, 0.0]
        self.cmd_gripper = 0.0
        self.is_active = False

        self.init_ui()
        self.connect_shm()

        # Timer 30ms (khoảng 30FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(30)

    def connect_shm(self):
        try:
            f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(f.fileno(), 0)
            self.status_label.setText("CONNECTED")
            self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 14px;")
        except FileNotFoundError:
            self.status_label.setText("DISCONNECTED (Run ROS2 Launch first!)")
            self.status_label.setStyleSheet("color: #F44336; font-weight: bold; font-size: 14px;")

    def init_ui(self):
        main_widget = QWidget()
        layout = QHBoxLayout()
        
        # ==========================
        # PANEL TRÁI: GIÁM SÁT (MONITOR)
        # ==========================
        left_panel = QVBoxLayout()
        
        # 1. Joint Monitor
        gb_joints = QGroupBox("Joint States (Feedback)")
        l_joints = QGridLayout()
        self.joint_labels = []
        for i in range(6):
            l_joints.addWidget(QLabel(f"Joint {i+1}:"), i, 0)
            
            pb = QProgressBar()
            pb.setRange(-314, 314) # -3.14 to 3.14 rad
            pb.setTextVisible(False)
            pb.setStyleSheet("QProgressBar::chunk { background-color: #2196F3; }")
            
            val = QLabel("0.00")
            val.setFixedWidth(50)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            
            l_joints.addWidget(pb, i, 1)
            l_joints.addWidget(val, i, 2)
            self.joint_labels.append((pb, val))
        gb_joints.setLayout(l_joints)
        left_panel.addWidget(gb_joints)

        # 2. End-Effector Monitor
        gb_ee = QGroupBox("End-Effector Pose (Feedback)")
        l_ee = QGridLayout()
        self.ee_labels = {}
        # Dùng tên đầy đủ để hiển thị
        monitor_tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for i, tag in enumerate(monitor_tags):
            lbl_title = QLabel(f"{tag}:")
            lbl_val = QLabel("0.000")
            lbl_val.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
            lbl_val.setStyleSheet("color: #FFC107;") # Màu vàng nổi bật
            
            row = i // 3
            col = (i % 3) * 2
            l_ee.addWidget(lbl_title, row, col)
            l_ee.addWidget(lbl_val, row, col + 1)
            self.ee_labels[tag] = lbl_val
        gb_ee.setLayout(l_ee)
        left_panel.addWidget(gb_ee)

        # ==========================
        # PANEL PHẢI: ĐIỀU KHIỂN (CONTROL)
        # ==========================
        right_panel = QVBoxLayout()

        # Nút kích hoạt chính
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(60)
        self.btn_active.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.btn_active.setStyleSheet("background-color: #546E7A; color: white; border-radius: 5px;")
        self.btn_active.clicked.connect(self.toggle_control)
        right_panel.addWidget(self.btn_active)

        # 1. Arm Control (Target Position)
        gb_arm = QGroupBox("Arm Target Control")
        gb_arm.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #757575; margin-top: 10px; }")
        l_arm = QGridLayout()
        
        self.arm_inputs = {}
        # SỬA LỖI: Dùng tên khác nhau cho Y và Yaw để không bị trùng key trong Dict
        self.control_tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        
        for i, tag in enumerate(self.control_tags):
            spin = QDoubleSpinBox()
            spin.setRange(-3.14, 3.14) # Giới hạn nhập liệu
            spin.setSingleStep(0.01)
            spin.setDecimals(3)
            spin.setEnabled(False) # Mặc định khóa
            spin.valueChanged.connect(self.update_arm_target)
            
            # Style cho ô nhập liệu
            spin.setStyleSheet("QDoubleSpinBox { padding: 5px; font-size: 14px; }")

            l_arm.addWidget(QLabel(tag), i, 0)
            l_arm.addWidget(spin, i, 1)
            self.arm_inputs[tag] = spin
        
        gb_arm.setLayout(l_arm)
        right_panel.addWidget(gb_arm)

        # 2. Gripper Control
        gb_grip = QGroupBox("Gripper Control")
        gb_grip.setStyleSheet("QGroupBox { border: 1px solid #FF9800; margin-top: 10px; } QGroupBox::title { color: #FF9800; }")
        l_grip = QVBoxLayout()
        
        # Slider
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 80) # 0.00 -> 0.80 rad
        self.grip_slider.setEnabled(False)
        self.grip_slider.valueChanged.connect(self.update_gripper_from_slider)
        
        # Spinbox hiển thị số
        self.grip_spin = QDoubleSpinBox()
        self.grip_spin.setRange(0.0, 0.8) 
        self.grip_spin.setSingleStep(0.01)
        self.grip_spin.setEnabled(False)
        self.grip_spin.valueChanged.connect(self.update_gripper_from_spin)

        h_grip = QHBoxLayout()
        h_grip.addWidget(QLabel("Open"))
        h_grip.addWidget(self.grip_slider)
        h_grip.addWidget(QLabel("Close"))
        
        l_grip.addLayout(h_grip)
        l_grip.addWidget(self.grip_spin)
        gb_grip.setLayout(l_grip)
        right_panel.addWidget(gb_grip)

        # Layout chính
        layout.addLayout(left_panel, 1)
        layout.addLayout(right_panel, 1)
        
        # Footer Status
        self.status_label = QLabel("INIT")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFixedHeight(30)
        
        main_box = QVBoxLayout()
        main_box.addLayout(layout)
        main_box.addWidget(self.status_label)
        main_widget.setLayout(main_box)
        self.setCentralWidget(main_widget)

    def toggle_control(self):
        self.is_active = self.btn_active.isChecked()
        if self.is_active:
            self.btn_active.setText("CONTROL ACTIVE")
            self.btn_active.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 5px;")
            # Mở khóa các ô nhập liệu
            for s in self.arm_inputs.values(): s.setEnabled(True)
            self.grip_slider.setEnabled(True)
            self.grip_spin.setEnabled(True)
        else:
            self.btn_active.setText("ENABLE CONTROL")
            self.btn_active.setStyleSheet("background-color: #546E7A; color: white; border-radius: 5px;")
            # Khóa lại
            for s in self.arm_inputs.values(): s.setEnabled(False)
            self.grip_slider.setEnabled(False)
            self.grip_spin.setEnabled(False)
            
            # Gửi tín hiệu dừng ngay lập tức
            if self.shm:
                self.shm.seek(200)
                self.shm.write(struct.pack('?', False))

    def update_gripper_from_slider(self, val):
        real_val = val / 100.0
        self.grip_spin.blockSignals(True)
        self.grip_spin.setValue(real_val)
        self.grip_spin.blockSignals(False)
        self.cmd_gripper = real_val

    def update_gripper_from_spin(self, val):
        int_val = int(val * 100)
        self.grip_slider.blockSignals(True)
        self.grip_slider.setValue(int_val)
        self.grip_slider.blockSignals(False)
        self.cmd_gripper = val

    def update_arm_target(self):
        # Cập nhật biến target từ các ô nhập liệu
        for i, tag in enumerate(self.control_tags):
            val = self.arm_inputs[tag].value()
            if i < 3: self.target_pos[i] = val
            else: self.target_rpy[i-3] = val

    def update_loop(self):
        if not self.shm: return

        try:
            # 1. ĐỌC DỮ LIỆU TỪ C++ (FEEDBACK)
            self.shm.seek(0)
            # 18 doubles = 144 bytes
            data_bytes = self.shm.read(144) 
            data = struct.unpack('18d', data_bytes)
            
            # Cập nhật thanh Joint
            for i in range(6):
                self.joint_labels[i][0].setValue(int(data[i]*100))
                self.joint_labels[i][1].setText(f"{data[i]:.2f}")
            
            # Cập nhật hiển thị EE Pose
            ee_vals = data[12:18] # Index 12-17
            monitor_tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            for i, tag in enumerate(monitor_tags):
                self.ee_labels[tag].setText(f"{ee_vals[i]:.3f}")

            # SYNC: Nếu chưa active, tự động điền giá trị hiện tại vào ô Target
            # Để khi bấm Enable, robot không bị giật
            if not self.is_active:
                for i, tag in enumerate(self.control_tags):
                    self.arm_inputs[tag].blockSignals(True)
                    self.arm_inputs[tag].setValue(ee_vals[i])
                    self.arm_inputs[tag].blockSignals(False)
                    # Cập nhật luôn biến nội bộ
                    if i < 3: self.target_pos[i] = ee_vals[i]
                    else: self.target_rpy[i-3] = ee_vals[i]

            # 2. GHI DỮ LIỆU XUỐNG C++ (COMMAND)
            if self.is_active:
                # Offset 144: 7 double (3 Pos + 3 RPY + 1 Grip)
                cmd_bytes = struct.pack('7d', 
                    self.target_pos[0], self.target_pos[1], self.target_pos[2],
                    self.target_rpy[0], self.target_rpy[1], self.target_rpy[2],
                    self.cmd_gripper) 
                
                self.shm.seek(144)
                self.shm.write(cmd_bytes)
                
                # Offset 200: Bool Active
                self.shm.seek(200)
                self.shm.write(struct.pack('?', True))

        except Exception as e:
            # print(f"Error: {e}") # Uncomment để debug nếu cần
            pass

    def closeEvent(self, event):
        if self.shm: 
            # Gửi tín hiệu tắt
            self.shm.seek(200)
            self.shm.write(struct.pack('?', False))
            self.shm.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Thiết lập giao diện tối (Dark Theme) cho chuyên nghiệp
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(40, 40, 40))
    p.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    p.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Button, QColor(60, 60, 60))
    p.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    p.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(p)

    win = RobotInterface()
    win.show()
    sys.exit(app.exec())