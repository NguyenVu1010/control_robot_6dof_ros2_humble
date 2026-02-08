from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QPushButton
from config import JOINT_KEYS

class JointTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout(self)
        
        gb = QGroupBox("Joint Jogging (Hold to Move)")
        grid = QGridLayout()
        for i, key in enumerate(JOINT_KEYS):
            btn_m = QPushButton(f"{key} -")
            btn_p = QPushButton(f"{key} +")
            # Kết nối sự kiện nhấn và nhả
            btn_m.pressed.connect(lambda idx=i: self.set_vel(idx, -0.2))
            btn_m.released.connect(lambda idx=i: self.set_vel(idx, 0.0))
            btn_p.pressed.connect(lambda idx=i: self.set_vel(idx, 0.2))
            btn_p.released.connect(lambda idx=i: self.set_vel(idx, 0.0))
            grid.addWidget(btn_m, i, 0); grid.addWidget(btn_p, i, 1)
        
        gb.setLayout(grid)
        layout.addWidget(gb)
        layout.addStretch()

    def set_vel(self, idx, val):
        if self.main_win.is_active:
            self.main_win.manual_vel[idx] = val