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
import matplotlib.gridspec as gridspec


class PlotTab(QWidget):
    # Số điểm tối đa lưu trữ
    MAX_POINTS = 3000  # ~90 giây ở 30Hz

    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        self.is_recording = False

        # Dữ liệu quỹ đạo thực tế (actual)
        self.data_x = deque(maxlen=self.MAX_POINTS)
        self.data_y = deque(maxlen=self.MAX_POINTS)
        self.data_z = deque(maxlen=self.MAX_POINTS)

        # Dữ liệu quỹ đạo mục tiêu (target)
        self.target_x = deque(maxlen=self.MAX_POINTS)
        self.target_y = deque(maxlen=self.MAX_POINTS)
        self.target_z = deque(maxlen=self.MAX_POINTS)

        # Dữ liệu sai lệch (error)
        self.err_total = deque(maxlen=self.MAX_POINTS)  # Euclidean distance (mm)
        self.err_x = deque(maxlen=self.MAX_POINTS)      # eX (mm)
        self.err_y = deque(maxlen=self.MAX_POINTS)      # eY (mm)
        self.err_z = deque(maxlen=self.MAX_POINTS)      # eZ (mm)

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
        self.canvas.setMinimumHeight(400)
        layout.addWidget(self.canvas, 1)

        # Thông tin điểm hiện tại
        self.lbl_info = QLabel("Điểm: 0 | Vị trí: ---")
        self.lbl_info.setStyleSheet("color: #B0BEC5; font-size: 11px;")
        layout.addWidget(self.lbl_info)

        # Tạo axes ban đầu
        self.current_mode = "3D"
        self._create_axes()

    def _create_axes(self):
        """Tạo lại axes theo chế độ hiển thị: trajectory trên, error dưới"""
        self.figure.clear()

        # Dùng GridSpec: trajectory chiếm 65%, error chiếm 35%
        gs = gridspec.GridSpec(2, 1, height_ratios=[65, 35], hspace=0.35)

        # --- Subplot trên: Quỹ đạo ---
        if self.current_mode == "3D":
            self.ax = self.figure.add_subplot(gs[0], projection='3d', facecolor='#1e1e1e')
            self.ax.set_xlabel('X (m)', color='white', fontsize=9)
            self.ax.set_ylabel('Y (m)', color='white', fontsize=9)
            self.ax.set_zlabel('Z (m)', color='white', fontsize=9)
            self.ax.tick_params(colors='white', labelsize=7)
        else:
            self.ax = self.figure.add_subplot(gs[0], facecolor='#1e1e1e')
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

        # --- Subplot dưới: Sai lệch ---
        self.ax_err = self.figure.add_subplot(gs[1], facecolor='#1e1e1e')
        self.ax_err.set_xlabel('Mẫu', color='white', fontsize=9)
        self.ax_err.set_ylabel('Sai lệch (mm)', color='white', fontsize=9)
        self.ax_err.set_title("Sai lệch bám quỹ đạo", color='white', fontsize=10, fontweight='bold')
        self.ax_err.tick_params(colors='white', labelsize=8)
        self.ax_err.grid(True, alpha=0.3, color='gray')

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
        old_tx, old_ty, old_tz = list(self.target_x), list(self.target_y), list(self.target_z)
        self.target_x = deque(old_tx, maxlen=val)
        self.target_y = deque(old_ty, maxlen=val)
        self.target_z = deque(old_tz, maxlen=val)
        old_et, old_ex, old_ey, old_ez = list(self.err_total), list(self.err_x), list(self.err_y), list(self.err_z)
        self.err_total = deque(old_et, maxlen=val)
        self.err_x = deque(old_ex, maxlen=val)
        self.err_y = deque(old_ey, maxlen=val)
        self.err_z = deque(old_ez, maxlen=val)

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
        self.target_x.clear()
        self.target_y.clear()
        self.target_z.clear()
        self.err_total.clear()
        self.err_x.clear()
        self.err_y.clear()
        self.err_z.clear()
        self._create_axes()

    def update_plot(self, ee_pos, target_pos=None):
        """
        Được gọi từ vòng lặp chính (30Hz).
        ee_pos: tuple/list (x, y, z) tọa độ điểm cuối thực tế
        target_pos: tuple/list (x, y, z) tọa độ mục tiêu lý tưởng (hoặc None)
        """
        x, y, z = ee_pos

        # Ghi dữ liệu nếu đang recording
        if self.is_recording:
            self.data_x.append(x)
            self.data_y.append(y)
            self.data_z.append(z)
            if target_pos:
                self.target_x.append(target_pos[0])
                self.target_y.append(target_pos[1])
                self.target_z.append(target_pos[2])
                # Tính và lưu sai lệch
                ex = (target_pos[0] - x) * 1000  # mm
                ey = (target_pos[1] - y) * 1000
                ez = (target_pos[2] - z) * 1000
                et = math.sqrt(ex**2 + ey**2 + ez**2)
                self.err_x.append(ex)
                self.err_y.append(ey)
                self.err_z.append(ez)
                self.err_total.append(et)

        # Cập nhật thông tin
        err_str = ""
        if target_pos:
            err = math.sqrt(sum((target_pos[i] - ee_pos[i])**2 for i in range(3)))
            err_str = f" | Err={err*1000:.1f}mm"
        self.lbl_info.setText(f"Điểm: {len(self.data_x)} | Vị trí: X={x:.3f}  Y={y:.3f}  Z={z:.3f}{err_str}")

        # Chỉ vẽ lại khi tab đang hiển thị (tiết kiệm CPU)
        if not self.isVisible():
            return

        self._redraw(current_pos=(x, y, z))

    def _redraw(self, current_pos=None):
        """Vẽ lại toàn bộ biểu đồ"""
        self.ax.clear()

        # Cấu hình lại axes trajectory
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

        # Vẽ quỹ đạo mục tiêu (target - nét đứt cam)
        has_target = len(self.target_x) > 1
        if has_target:
            tx = list(self.target_x)
            ty = list(self.target_y)
            tz = list(self.target_z)

            if self.current_mode == "3D":
                self.ax.plot3D(tx, ty, tz, color='#FF7043', linewidth=1.5, linestyle='--', alpha=0.9, label='Target')
            else:
                td1, td2 = self._get_2d_data(tx, ty, tz)
                self.ax.plot(td1, td2, color='#FF7043', linewidth=1.5, linestyle='--', alpha=0.9, label='Target')

        # Vẽ quỹ đạo thực tế (actual - xanh dương)
        if len(self.data_x) > 1:
            lx = list(self.data_x)
            ly = list(self.data_y)
            lz = list(self.data_z)

            if self.current_mode == "3D":
                self.ax.plot3D(lx, ly, lz, color='#42A5F5', linewidth=1.2, alpha=0.8, label='Actual')
                # Điểm đầu (xanh lá) và điểm cuối (đỏ)
                self.ax.scatter(*[[lx[0]], [ly[0]], [lz[0]]], color='#66BB6A', s=40, zorder=5)
                self.ax.scatter(*[[lx[-1]], [ly[-1]], [lz[-1]]], color='#EF5350', s=40, zorder=5)
            else:
                d1, d2 = self._get_2d_data(lx, ly, lz)
                self.ax.plot(d1, d2, color='#42A5F5', linewidth=1.2, alpha=0.8, label='Actual')
                self.ax.plot(d1[0], d2[0], 'o', color='#66BB6A', markersize=7, zorder=5)
                self.ax.plot(d1[-1], d2[-1], 'o', color='#EF5350', markersize=7, zorder=5)

        # Legend (chỉ hiện khi có cả 2 đường)
        if has_target and len(self.data_x) > 1:
            self.ax.legend(loc='upper right', fontsize=8, facecolor='#333333',
                          edgecolor='gray', labelcolor='white')

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

        # Đặt tỉ lệ trục 1:1 (1:1:1 cho 3D)
        if self.current_mode == "3D":
            # Thu thập tất cả dữ liệu để tính range
            all_x, all_y, all_z = [], [], []
            if len(self.data_x) > 0:
                all_x += list(self.data_x); all_y += list(self.data_y); all_z += list(self.data_z)
            if len(self.target_x) > 0:
                all_x += list(self.target_x); all_y += list(self.target_y); all_z += list(self.target_z)
            if current_pos:
                all_x.append(current_pos[0]); all_y.append(current_pos[1]); all_z.append(current_pos[2])
            if all_x:
                mid_x = (max(all_x) + min(all_x)) / 2
                mid_y = (max(all_y) + min(all_y)) / 2
                mid_z = (max(all_z) + min(all_z)) / 2
                max_range = max(max(all_x)-min(all_x), max(all_y)-min(all_y), max(all_z)-min(all_z), 0.01) / 2
                self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
                self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
                self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        else:
            self.ax.set_aspect('equal', adjustable='datalim')

        # --- VẼ ĐỒ THỊ SAI LỆCH (subplot dưới) ---
        self.ax_err.clear()
        self.ax_err.set_facecolor('#1e1e1e')
        self.ax_err.set_xlabel('Mẫu', color='white', fontsize=9)
        self.ax_err.set_ylabel('Sai lệch (mm)', color='white', fontsize=9)
        self.ax_err.tick_params(colors='white', labelsize=8)
        self.ax_err.grid(True, alpha=0.3, color='gray')

        if len(self.err_total) > 1:
            samples = list(range(len(self.err_total)))
            et = list(self.err_total)
            ex = list(self.err_x)
            ey = list(self.err_y)
            ez = list(self.err_z)

            # Vẽ sai lệch từng trục (mờ hơn)
            self.ax_err.plot(samples, ex, color='#EF5350', linewidth=0.8, alpha=0.6, label='eX')
            self.ax_err.plot(samples, ey, color='#66BB6A', linewidth=0.8, alpha=0.6, label='eY')
            self.ax_err.plot(samples, ez, color='#42A5F5', linewidth=0.8, alpha=0.6, label='eZ')
            # Vẽ sai lệch tổng (đậm, nổi bật)
            self.ax_err.plot(samples, et, color='#FFD740', linewidth=1.5, alpha=0.95, label='Total')

            # Tính thống kê
            mean_err = sum(et) / len(et)
            max_err = max(et)
            rms_err = math.sqrt(sum(e**2 for e in et) / len(et))

            # Đường trung bình (nét đứt trắng)
            self.ax_err.axhline(y=mean_err, color='white', linewidth=0.8, linestyle='--', alpha=0.5)

            title_str = f"Sai lệch  |  Mean={mean_err:.1f}mm  Max={max_err:.1f}mm  RMS={rms_err:.1f}mm"
            self.ax_err.set_title(title_str, color='white', fontsize=9, fontweight='bold')

            self.ax_err.legend(loc='upper right', fontsize=7, facecolor='#333333',
                              edgecolor='gray', labelcolor='white', ncol=4)
        else:
            self.ax_err.set_title("Sai lệch bám quỹ đạo (chưa có dữ liệu)",
                                  color='#666666', fontsize=9)

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
