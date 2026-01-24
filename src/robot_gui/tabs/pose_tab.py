from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QLabel, QDoubleSpinBox, QPushButton
from config import COORD_KEYS

class PoseTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        
        gb = QGroupBox("Target Cartesian Pose")
        grid = QGridLayout()
        self.spins = {}
        for i, key in enumerate(COORD_KEYS):
            grid.addWidget(QLabel(key + ":"), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.005) # Độ chia 0.01 theo yêu cầu
            spin.valueChanged.connect(self.on_change)
            grid.addWidget(spin, i, 1)
            self.spins[key] = spin
        gb.setLayout(grid)
        layout.addWidget(gb)
        
        # Bỏ nút Sync vì bây giờ chúng ta sẽ làm tự động
        self.setLayout(layout)

    def on_change(self):
        # Chỉ cập nhật vào biến điều khiển khi hệ thống đang Active
        if self.main_win.is_active:
            for i, key in enumerate(COORD_KEYS):
                val = self.spins[key].value()
                if i < 3: self.main_win.target_pos[i] = val
                else: self.main_win.target_rpy[i-3] = val

    def update_ui_values(self, pos, rpy):
        """Hàm này dùng để main_loop đẩy dữ liệu thực tế vào ô nhập liệu"""
        self.blockSignals(True) # Quan trọng: Chặn tín hiệu để không gọi ngược lại hàm on_change
        for i, key in enumerate(COORD_KEYS):
            val = pos[i] if i < 3 else rpy[i-3]
            self.spins[key].setValue(val)
        self.blockSignals(False)