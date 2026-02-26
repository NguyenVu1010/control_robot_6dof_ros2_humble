import time
import math
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QGroupBox, QGridLayout, 
                             QPushButton, QHBoxLayout, QDoubleSpinBox, QLabel, QComboBox)
from config import COORD_KEYS

class VelocityTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout(self)
        
        self.ee_vel_local = [0.0] * 6 
        self.test_mode = None 
        self.start_t = 0

        # --- PHẦN 1: TÂM VÀ HƯỚNG (CENTER & ORIENTATION) ---
        gb_orient = QGroupBox("Target Center & Orientation")
        orient_grid = QGridLayout()

        # Tọa độ tâm
        for i, axis in enumerate(['X', 'Y', 'Z']):
            orient_grid.addWidget(QLabel(f"Center {axis} (m):"), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-2.0, 2.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            # Khởi tạo bằng tọa độ hiện tại của robot để tiện sử dụng
            orient_grid.addWidget(spin, i, 1)
            setattr(self, f"spin_center_{axis.lower()}", spin)

        # Chọn mặt phẳng cho hình tròn / Trục cho đường thẳng
        orient_grid.addWidget(QLabel("Circle Plane:"), 0, 2)
        self.combo_plane = QComboBox()
        self.combo_plane.addItems(["XY Plane", "YZ Plane", "XZ Plane"])
        orient_grid.addWidget(self.combo_plane, 0, 3)

        orient_grid.addWidget(QLabel("Line Axis:"), 1, 2)
        self.combo_axis = QComboBox()
        self.combo_axis.addItems(["X Axis", "Y Axis", "Z Axis"])
        orient_grid.addWidget(self.combo_axis, 1, 3)

        gb_orient.setLayout(orient_grid)
        layout.addWidget(gb_orient)

        # --- PHẦN 2: THÔNG SỐ ĐỘNG LỰC HỌC (SPEED/RADIUS) ---
        gb_settings = QGroupBox("Motion Parameters")
        settings_grid = QGridLayout()

        settings_grid.addWidget(QLabel("Circle Radius (m):"), 0, 0)
        self.spin_circle_r = QDoubleSpinBox()
        self.spin_circle_r.setRange(0.01, 0.5); self.spin_circle_r.setValue(0.05)
        settings_grid.addWidget(self.spin_circle_r, 0, 1)

        settings_grid.addWidget(QLabel("Circle Speed (rad/s):"), 0, 2)
        self.spin_circle_w = QDoubleSpinBox()
        self.spin_circle_w.setRange(0.1, 5.0); self.spin_circle_w.setValue(1.0)
        settings_grid.addWidget(self.spin_circle_w, 0, 3)

        settings_grid.addWidget(QLabel("Line Speed (m/s):"), 1, 0)
        self.spin_line_speed = QDoubleSpinBox()
        self.spin_line_speed.setRange(0.001, 0.2); self.spin_line_speed.setValue(0.03)
        settings_grid.addWidget(self.spin_line_speed, 1, 1)

        gb_settings.setLayout(settings_grid)
        layout.addWidget(gb_settings)

        # --- PHẦN 3: NÚT ĐIỀU KHIỂN ---
        gb_ctrl = QGroupBox("Execution")
        ctrl_layout = QHBoxLayout()
        
        self.btn_circle = QPushButton("START CIRCLE TEST")
        self.btn_circle.setCheckable(True)
        self.btn_circle.toggled.connect(lambda ch: self.start_test('circle', ch))
        
        self.btn_line = QPushButton("START LINE TEST")
        self.btn_line.setCheckable(True)
        self.btn_line.toggled.connect(lambda ch: self.start_test('line', ch))
        
        self.btn_sync_center = QPushButton("SYNC CENTER TO ACTUAL")
        self.btn_sync_center.clicked.connect(self.sync_center)

        ctrl_layout.addWidget(self.btn_circle)
        ctrl_layout.addWidget(self.btn_line)
        ctrl_layout.addWidget(self.btn_sync_center)
        gb_ctrl.setLayout(ctrl_layout)
        layout.addWidget(gb_ctrl)

        layout.addStretch()

    def sync_center(self):
        """Lấy tọa độ hiện tại của robot nạp vào các ô Center"""
        fb_pose = self.main_win.last_fb_pose
        self.spin_center_x.setValue(fb_pose[0])
        self.spin_center_y.setValue(fb_pose[1])
        self.spin_center_z.setValue(fb_pose[2])

    def start_test(self, mode, active):
        if active:
            self.test_mode = mode
            self.start_t = time.time()
            if mode == 'circle': self.btn_line.setChecked(False)
            else: self.btn_circle.setChecked(False)
        else:
            self.test_mode = None

    def get_current_vel(self):
        """
        Trả về CHỈ feedforward velocity (không có position feedback).
        Position feedback sẽ được thực hiện ở Controller C++ tại 500Hz
        thay vì ở GUI 30Hz để giảm sai lệch bám đường.
        """
        if not self.test_mode:
            return [0.0]*6

        t = time.time() - self.start_t

        # --- CHẾ ĐỘ ĐƯỜNG TRÒN ---
        if self.test_mode == 'circle':
            r = self.spin_circle_r.value()
            w = self.spin_circle_w.value()
            plane = self.combo_plane.currentText()

            # Chỉ trả về vận tốc tiếp tuyến (Feedforward) - KHÔNG có Kp feedback
            if plane == "XY Plane":
                vx_ff = -r * w * math.sin(w * t)
                vy_ff =  r * w * math.cos(w * t)
                return [vx_ff, vy_ff, 0.0, 0.0, 0.0, 0.0]
            elif plane == "YZ Plane":
                vy_ff = -r * w * math.sin(w * t)
                vz_ff =  r * w * math.cos(w * t)
                return [0.0, vy_ff, vz_ff, 0.0, 0.0, 0.0]
            else:  # XZ Plane
                vx_ff = -r * w * math.sin(w * t)
                vz_ff =  r * w * math.cos(w * t)
                return [vx_ff, 0.0, vz_ff, 0.0, 0.0, 0.0]

        # --- CHẾ ĐỘ ĐƯỜNG THẲNG ---
        if self.test_mode == 'line':
            speed = self.spin_line_speed.value()
            axis = self.combo_axis.currentText()

            # Chỉ feedforward: speed * cos(t)
            v_ff = speed * math.cos(t)

            if axis == "X Axis": return [v_ff, 0, 0, 0, 0, 0]
            if axis == "Y Axis": return [0, v_ff, 0, 0, 0, 0]
            return [0, 0, v_ff, 0, 0, 0]

        return [0.0]*6

    def get_target_pos(self):
        """Trả về vị trí mục tiêu lý tưởng trên quỹ đạo (để vẽ đường target)"""
        if not self.test_mode:
            return None

        t = time.time() - self.start_t

        if self.test_mode == 'circle':
            r = self.spin_circle_r.value()
            w = self.spin_circle_w.value()
            cx = self.spin_center_x.value()
            cy = self.spin_center_y.value()
            cz = self.spin_center_z.value()
            plane = self.combo_plane.currentText()

            if plane == "XY Plane":
                return [cx + r * math.cos(w * t), cy + r * math.sin(w * t), cz]
            elif plane == "YZ Plane":
                return [cx, cy + r * math.cos(w * t), cz + r * math.sin(w * t)]
            else:  # XZ Plane
                return [cx + r * math.cos(w * t), cy, cz + r * math.sin(w * t)]

        if self.test_mode == 'line':
            speed = self.spin_line_speed.value()
            axis = self.combo_axis.currentText()
            cx = self.spin_center_x.value()
            cy = self.spin_center_y.value()
            cz = self.spin_center_z.value()
            displacement = speed * math.sin(t)

            if axis == "X Axis": return [cx + displacement, cy, cz]
            if axis == "Y Axis": return [cx, cy + displacement, cz]
            return [cx, cy, cz + displacement]

        return None