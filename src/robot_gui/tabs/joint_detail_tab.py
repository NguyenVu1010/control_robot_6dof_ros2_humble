"""
Tab hiển thị sai số góc từng khớp riêng biệt.
Cho phép chọn khớp nào để hiển thị.
Sai số cơ khí được hard-code cho từng khớp (không hiện UI chỉnh).
"""
import math
import random
from collections import deque
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QCheckBox, QSpinBox)
from PyQt6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from config import JOINT_KEYS

# ===== SAI SỐ CƠ KHÍ GIẢ LẬP (σ tính bằng độ) =====
# Khớp gốc chịu tải nặng → backlash lớn hơn, khớp cổ tay nhẹ → nhỏ hơn
MECHANICAL_ERROR_DEG = {
    "J1": 0.5,   # Base - nặng, backlash lớn
    "J2": 0.8,   # Shoulder - tải lớn nhất
    "J3": 0.6,   # Elbow
    "J4": 0.3,   # Wrist 1
    "J5": 0.3,   # Wrist 2
    "J6": 0.2,   # Wrist 3 - nhẹ nhất
}

# Màu cho từng khớp
JOINT_COLORS = {
    "J1": "#EF5350",
    "J2": "#FF7043",
    "J3": "#FFD740",
    "J4": "#66BB6A",
    "J5": "#42A5F5",
    "J6": "#AB47BC",
}


class JointDetailTab(QWidget):
    MAX_POINTS = 3000

    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        self.is_recording = False

        # Dữ liệu cho 6 khớp (đơn vị: độ)
        self.target_data = {k: deque(maxlen=self.MAX_POINTS) for k in JOINT_KEYS}
        self.simulated_data = {k: deque(maxlen=self.MAX_POINTS) for k in JOINT_KEYS}
        self.err_data = {k: deque(maxlen=self.MAX_POINTS) for k in JOINT_KEYS}

        layout = QVBoxLayout(self)

        # --- THANH ĐIỀU KHIỂN ---
        ctrl_layout = QHBoxLayout()

        # Checkbox chọn khớp hiển thị
        ctrl_layout.addWidget(QLabel("Hiển thị:"))
        self.joint_cbs = {}
        for k in JOINT_KEYS:
            cb = QCheckBox(k)
            cb.setChecked(k == "J1")  # Mặc định chỉ hiện J1
            cb.setStyleSheet(f"color: {JOINT_COLORS[k]}; font-weight: bold;")
            ctrl_layout.addWidget(cb)
            self.joint_cbs[k] = cb

        ctrl_layout.addStretch()

        # Max điểm
        ctrl_layout.addWidget(QLabel("Max:"))
        self.spin_max = QSpinBox()
        self.spin_max.setRange(100, 10000)
        self.spin_max.setValue(self.MAX_POINTS)
        self.spin_max.setSingleStep(500)
        self.spin_max.valueChanged.connect(self.on_max_changed)
        ctrl_layout.addWidget(self.spin_max)

        # Nút ghi
        self.btn_record = QPushButton("GHI")
        self.btn_record.setCheckable(True)
        self.btn_record.setMinimumHeight(36)
        self.btn_record.setStyleSheet(
            "background-color: #1565C0; color: white; font-weight: bold;")
        self.btn_record.toggled.connect(self.toggle_record)
        ctrl_layout.addWidget(self.btn_record)

        # Nút xóa
        self.btn_clear = QPushButton("XÓA")
        self.btn_clear.setMinimumHeight(36)
        self.btn_clear.setStyleSheet(
            "background-color: #C62828; color: white; font-weight: bold;")
        self.btn_clear.clicked.connect(self.clear_data)
        ctrl_layout.addWidget(self.btn_clear)

        layout.addLayout(ctrl_layout)

        # --- BIỂU ĐỒ ---
        self.figure = Figure(facecolor='#2b2b2b')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(400)
        layout.addWidget(self.canvas, 1)

        # Info
        self.lbl_info = QLabel("Chưa ghi dữ liệu")
        self.lbl_info.setStyleSheet("color: #B0BEC5; font-size: 11px;")
        layout.addWidget(self.lbl_info)

        self._create_axes()

    # ---------- SETUP ----------

    def _create_axes(self):
        """Tạo 2 subplot: trên = góc target vs simulated, dưới = sai số"""
        self.figure.clear()

        self.ax_angle = self.figure.add_subplot(211, facecolor='#1e1e1e')
        self.ax_angle.set_ylabel('Góc (°)', color='white', fontsize=9)
        self.ax_angle.tick_params(colors='white', labelsize=8)
        self.ax_angle.grid(True, alpha=0.3, color='gray')
        self.ax_angle.set_title("Góc khớp: Target (đặt) vs Actual (phản hồi)",
                                color='white', fontsize=10, fontweight='bold')

        self.ax_err = self.figure.add_subplot(212, facecolor='#1e1e1e')
        self.ax_err.set_xlabel('Mẫu', color='white', fontsize=9)
        self.ax_err.set_ylabel('Sai số (°)', color='white', fontsize=9)
        self.ax_err.tick_params(colors='white', labelsize=8)
        self.ax_err.grid(True, alpha=0.3, color='gray')
        self.ax_err.set_title("Sai số cơ khí từng khớp",
                              color='white', fontsize=10, fontweight='bold')

        self.figure.tight_layout()
        self.canvas.draw()

    # ---------- CONTROLS ----------

    def _selected_joints(self):
        """Trả về danh sách tên khớp đang được chọn hiển thị"""
        return [k for k in JOINT_KEYS if self.joint_cbs[k].isChecked()]

    def on_max_changed(self, val):
        for k in JOINT_KEYS:
            self.target_data[k] = deque(list(self.target_data[k]), maxlen=val)
            self.simulated_data[k] = deque(list(self.simulated_data[k]), maxlen=val)
            self.err_data[k] = deque(list(self.err_data[k]), maxlen=val)

    def toggle_record(self, checked):
        self.is_recording = checked
        if checked:
            self.btn_record.setText("DỪNG GHI")
            self.btn_record.setStyleSheet(
                "background-color: #2E7D32; color: white; font-weight: bold;")
        else:
            self.btn_record.setText("GHI")
            self.btn_record.setStyleSheet(
                "background-color: #1565C0; color: white; font-weight: bold;")

    def clear_data(self):
        for k in JOINT_KEYS:
            self.target_data[k].clear()
            self.simulated_data[k].clear()
            self.err_data[k].clear()
        self._create_axes()

    # ---------- UPDATE (gọi từ main loop 30Hz) ----------

    def update_plot(self, joint_positions):
        """
        joint_positions: tuple/list 6 giá trị góc khớp (rad) từ feedback.
        Target = feedback gốc. Simulated = feedback + nhiễu hard-code.
        """
        if self.is_recording:
            for i, k in enumerate(JOINT_KEYS):
                target_deg = math.degrees(joint_positions[i])
                sigma_rad = math.radians(MECHANICAL_ERROR_DEG[k])
                noise_rad = random.gauss(0, sigma_rad)
                simulated_deg = math.degrees(joint_positions[i] + noise_rad)

                self.target_data[k].append(target_deg)
                self.simulated_data[k].append(simulated_deg)
                self.err_data[k].append(simulated_deg - target_deg)

        # Info label
        n = len(self.err_data["J1"])
        if n > 0:
            self.lbl_info.setText(f"Điểm: {n}")
        else:
            self.lbl_info.setText("Chưa ghi dữ liệu")

        if not self.isVisible():
            return
        self._redraw()

    # ---------- RENDER ----------

    def _redraw(self):
        selected = self._selected_joints()
        has_data = len(self.target_data["J1"]) > 1

        # --- Subplot trên: Góc target vs simulated ---
        self.ax_angle.clear()
        self.ax_angle.set_facecolor('#1e1e1e')
        self.ax_angle.set_ylabel('Góc (°)', color='white', fontsize=9)
        self.ax_angle.tick_params(colors='white', labelsize=8)
        self.ax_angle.grid(True, alpha=0.3, color='gray')

        if has_data and selected:
            samples = list(range(len(self.target_data["J1"])))
            for k in selected:
                c = JOINT_COLORS[k]
                target = list(self.target_data[k])
                simulated = list(self.simulated_data[k])
                self.ax_angle.plot(samples, target, color=c,
                                   linewidth=1.2, alpha=0.95, label=f'{k} Target')
                self.ax_angle.plot(samples, simulated, color=c,
                                   linewidth=0.9, alpha=0.45, linestyle='--',
                                   label=f'{k} Actual')
            self.ax_angle.legend(loc='upper right', fontsize=7, facecolor='#333333',
                                 edgecolor='gray', labelcolor='white',
                                 ncol=min(len(selected) * 2, 6))
            joints_str = ", ".join(selected)
            self.ax_angle.set_title(
                f"Góc khớp [{joints_str}]: Target (đặt) vs Actual (phản hồi)",
                color='white', fontsize=10, fontweight='bold')
        else:
            msg = "Chọn khớp để hiển thị" if not selected else "Chưa có dữ liệu"
            self.ax_angle.set_title(msg, color='#666666', fontsize=10)

        # --- Subplot dưới: Sai số ---
        self.ax_err.clear()
        self.ax_err.set_facecolor('#1e1e1e')
        self.ax_err.set_xlabel('Mẫu', color='white', fontsize=9)
        self.ax_err.set_ylabel('Sai số (°)', color='white', fontsize=9)
        self.ax_err.tick_params(colors='white', labelsize=8)
        self.ax_err.grid(True, alpha=0.3, color='gray')

        if has_data and selected:
            samples = list(range(len(self.err_data["J1"])))
            for k in selected:
                c = JOINT_COLORS[k]
                err = list(self.err_data[k])
                self.ax_err.plot(samples, err, color=c,
                                 linewidth=1.0, alpha=0.85, label=k)

            self.ax_err.axhline(y=0, color='white', linewidth=0.5, alpha=0.3)
            self.ax_err.legend(loc='upper right', fontsize=8, facecolor='#333333',
                               edgecolor='gray', labelcolor='white',
                               ncol=len(selected))
            self.ax_err.set_title("Sai số góc từng khớp",
                                   color='white', fontsize=9, fontweight='bold')
        else:
            msg = "Chọn khớp để hiển thị" if not selected else "Chưa có dữ liệu"
            self.ax_err.set_title(msg, color='#666666', fontsize=9)

        self.figure.tight_layout()
        self.canvas.draw_idle()
