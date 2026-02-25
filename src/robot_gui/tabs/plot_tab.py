"""
Tab vẽ biểu đồ quỹ đạo điểm cuối (End-Effector) 2D/3D
Sử dụng matplotlib nhúng trong PyQt6
"""
import math
from collections import deque
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                             QPushButton, QComboBox, QLabel, QSpinBox)
from PyQt6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PlotTab(QWidget):
    # Số điểm tối đa lưu trữ
    MAX_POINTS = 3000  # ~90 giây ở 30Hz

    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        self.is_recording = False

        # Dữ liệu quỹ đạo
        self.data_x = deque(maxlen=self.MAX_POINTS)
        self.data_y = deque(maxlen=self.MAX_POINTS)
        self.data_z = deque(maxlen=self.MAX_POINTS)

        layout = QVBoxLayout(self)

        # --- THANH ĐIỀU KHIỂN ---
        ctrl_layout = QHBoxLayout()

        # Chọn chế độ hiển thị
        ctrl_layout.addWidget(QLabel("Chế độ:"))
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["3D", "XY", "XZ", "YZ"])
        self.combo_mode.currentIndexChanged.connect(self.on_mode_changed)
        ctrl_layout.addWidget(self.combo_mode)

        # Số điểm hiển thị tối đa
        ctrl_layout.addWidget(QLabel("Max điểm:"))
        self.spin_max = QSpinBox()
        self.spin_max.setRange(100, 10000)
        self.spin_max.setValue(self.MAX_POINTS)
        self.spin_max.setSingleStep(500)
        self.spin_max.valueChanged.connect(self.on_max_changed)
        ctrl_layout.addWidget(self.spin_max)

        ctrl_layout.addStretch()

        # Nút ghi / dừng
        self.btn_record = QPushButton("GHI QUỸ ĐẠO")
        self.btn_record.setCheckable(True)
        self.btn_record.setMinimumHeight(36)
        self.btn_record.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")
        self.btn_record.toggled.connect(self.toggle_record)
        ctrl_layout.addWidget(self.btn_record)

        # Nút xóa
        self.btn_clear = QPushButton("XÓA")
        self.btn_clear.setMinimumHeight(36)
        self.btn_clear.setStyleSheet("background-color: #C62828; color: white; font-weight: bold;")
        self.btn_clear.clicked.connect(self.clear_data)
        ctrl_layout.addWidget(self.btn_clear)

        layout.addLayout(ctrl_layout)

        # --- BIỂU ĐỒ MATPLOTLIB ---
        self.figure = Figure(facecolor='#2b2b2b')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(300)
        layout.addWidget(self.canvas, 1)

        # Thông tin điểm hiện tại
        self.lbl_info = QLabel("Điểm: 0 | Vị trí: ---")
        self.lbl_info.setStyleSheet("color: #B0BEC5; font-size: 11px;")
        layout.addWidget(self.lbl_info)

        # Tạo axes ban đầu
        self.current_mode = "3D"
        self._create_axes()

    def _create_axes(self):
        """Tạo lại axes theo chế độ hiển thị"""
        self.figure.clear()
        if self.current_mode == "3D":
            self.ax = self.figure.add_subplot(111, projection='3d', facecolor='#1e1e1e')
            self.ax.set_xlabel('X (m)', color='white', fontsize=9)
            self.ax.set_ylabel('Y (m)', color='white', fontsize=9)
            self.ax.set_zlabel('Z (m)', color='white', fontsize=9)
            self.ax.tick_params(colors='white', labelsize=7)
        else:
            self.ax = self.figure.add_subplot(111, facecolor='#1e1e1e')
            if self.current_mode == "XY":
                self.ax.set_xlabel('X (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Y (m)', color='white', fontsize=9)
            elif self.current_mode == "XZ":
                self.ax.set_xlabel('X (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Z (m)', color='white', fontsize=9)
            else:  # YZ
                self.ax.set_xlabel('Y (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Z (m)', color='white', fontsize=9)
            self.ax.tick_params(colors='white', labelsize=8)
            self.ax.grid(True, alpha=0.3, color='gray')

        self.ax.set_title(f"Quỹ đạo End-Effector ({self.current_mode})",
                          color='white', fontsize=11, fontweight='bold')
        self.figure.tight_layout()
        self.canvas.draw()

    def on_mode_changed(self, idx):
        modes = ["3D", "XY", "XZ", "YZ"]
        self.current_mode = modes[idx]
        self._create_axes()
        self._redraw()

    def on_max_changed(self, val):
        # Tạo deque mới với maxlen mới, giữ lại dữ liệu cũ
        old_x, old_y, old_z = list(self.data_x), list(self.data_y), list(self.data_z)
        self.data_x = deque(old_x, maxlen=val)
        self.data_y = deque(old_y, maxlen=val)
        self.data_z = deque(old_z, maxlen=val)

    def toggle_record(self, checked):
        self.is_recording = checked
        if checked:
            self.btn_record.setText("DỪNG GHI")
            self.btn_record.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
        else:
            self.btn_record.setText("GHI QUỸ ĐẠO")
            self.btn_record.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")

    def clear_data(self):
        self.data_x.clear()
        self.data_y.clear()
        self.data_z.clear()
        self._create_axes()

    def update_plot(self, ee_pos):
        """
        Được gọi từ vòng lặp chính (30Hz).
        ee_pos: tuple/list (x, y, z) tọa độ điểm cuối
        """
        x, y, z = ee_pos

        # Ghi dữ liệu nếu đang recording
        if self.is_recording:
            self.data_x.append(x)
            self.data_y.append(y)
            self.data_z.append(z)

        # Cập nhật thông tin
        self.lbl_info.setText(f"Điểm: {len(self.data_x)} | Vị trí: X={x:.3f}  Y={y:.3f}  Z={z:.3f}")

        # Chỉ vẽ lại khi tab đang hiển thị (tiết kiệm CPU)
        if not self.isVisible():
            return

        self._redraw(current_pos=(x, y, z))

    def _redraw(self, current_pos=None):
        """Vẽ lại toàn bộ biểu đồ"""
        self.ax.clear()

        # Cấu hình lại axes
        if self.current_mode == "3D":
            self.ax.set_xlabel('X (m)', color='white', fontsize=9)
            self.ax.set_ylabel('Y (m)', color='white', fontsize=9)
            self.ax.set_zlabel('Z (m)', color='white', fontsize=9)
            self.ax.tick_params(colors='white', labelsize=7)
            self.ax.set_facecolor('#1e1e1e')
        else:
            if self.current_mode == "XY":
                self.ax.set_xlabel('X (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Y (m)', color='white', fontsize=9)
            elif self.current_mode == "XZ":
                self.ax.set_xlabel('X (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Z (m)', color='white', fontsize=9)
            else:
                self.ax.set_xlabel('Y (m)', color='white', fontsize=9)
                self.ax.set_ylabel('Z (m)', color='white', fontsize=9)
            self.ax.tick_params(colors='white', labelsize=8)
            self.ax.grid(True, alpha=0.3, color='gray')
            self.ax.set_facecolor('#1e1e1e')

        self.ax.set_title(f"Quỹ đạo End-Effector ({self.current_mode})",
                          color='white', fontsize=11, fontweight='bold')

        # Vẽ quỹ đạo đã ghi
        if len(self.data_x) > 1:
            lx = list(self.data_x)
            ly = list(self.data_y)
            lz = list(self.data_z)

            if self.current_mode == "3D":
                self.ax.plot3D(lx, ly, lz, color='#42A5F5', linewidth=1.2, alpha=0.8)
                # Điểm đầu (xanh lá) và điểm cuối (đỏ)
                self.ax.scatter(*[[lx[0]], [ly[0]], [lz[0]]], color='#66BB6A', s=40, zorder=5)
                self.ax.scatter(*[[lx[-1]], [ly[-1]], [lz[-1]]], color='#EF5350', s=40, zorder=5)
            else:
                d1, d2 = self._get_2d_data(lx, ly, lz)
                self.ax.plot(d1, d2, color='#42A5F5', linewidth=1.2, alpha=0.8)
                self.ax.plot(d1[0], d2[0], 'o', color='#66BB6A', markersize=7, zorder=5)
                self.ax.plot(d1[-1], d2[-1], 'o', color='#EF5350', markersize=7, zorder=5)

        # Vẽ vị trí hiện tại (vàng)
        if current_pos:
            cx, cy, cz = current_pos
            if self.current_mode == "3D":
                self.ax.scatter([cx], [cy], [cz], color='#FFD740', s=60,
                                marker='D', zorder=10, edgecolors='white', linewidth=0.5)
            else:
                p1, p2 = self._get_2d_point(cx, cy, cz)
                self.ax.plot(p1, p2, 'D', color='#FFD740', markersize=8,
                             zorder=10, markeredgecolor='white', markeredgewidth=0.5)

        # Đặt tỉ lệ trục bằng nhau
        if self.current_mode != "3D":
            self.ax.set_aspect('equal', adjustable='datalim')

        self.figure.tight_layout()
        self.canvas.draw_idle()

    def _get_2d_data(self, lx, ly, lz):
        """Trả về 2 chuỗi dữ liệu cho biểu đồ 2D tùy theo mặt phẳng"""
        if self.current_mode == "XY":
            return lx, ly
        elif self.current_mode == "XZ":
            return lx, lz
        else:  # YZ
            return ly, lz

    def _get_2d_point(self, x, y, z):
        """Trả về 2 tọa độ cho 1 điểm theo mặt phẳng 2D"""
        if self.current_mode == "XY":
            return x, y
        elif self.current_mode == "XZ":
            return x, z
        else:
            return y, z
