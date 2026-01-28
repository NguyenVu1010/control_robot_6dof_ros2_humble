from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QLabel, QDoubleSpinBox, QPushButton, QMessageBox
from config import COORD_KEYS, MODE_TRAJECTORY

class PoseTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        
        # --- TARGET INPUTS (ĐỢI LỆNH GO) ---
        gb_input = QGroupBox("Coordinate Input (Set Target)")
        grid = QGridLayout()
        self.spins = {}
        for i, key in enumerate(COORD_KEYS):
            grid.addWidget(QLabel(key + ":"), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            grid.addWidget(spin, i, 1)
            self.spins[key] = spin
        
        self.btn_go = QPushButton("GO / EXECUTE")
        self.btn_go.setMinimumHeight(40)
        self.btn_go.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
        self.btn_go.clicked.connect(self.execute_pose)
        grid.addWidget(self.btn_go, len(COORD_KEYS), 0, 1, 2)
        gb_input.setLayout(grid)
        layout.addWidget(gb_input)

        # --- JOGGING BUTTONS (CHẠY TỨC THÌ) ---
        gb_jog = QGroupBox("Jogging (Instant Move 0.01)")
        jog_grid = QGridLayout()
        for i, key in enumerate(COORD_KEYS):
            btn_minus = QPushButton(f"{key} -")
            btn_plus = QPushButton(f"{key} +")
            btn_minus.clicked.connect(lambda ch, k=key, d=-0.01: self.jog_step(k, d))
            btn_plus.clicked.connect(lambda ch, k=key, d=0.01: self.jog_step(k, d))
            jog_grid.addWidget(btn_minus, i, 0)
            jog_grid.addWidget(btn_plus, i, 1)
        gb_jog.setLayout(jog_grid)
        layout.addWidget(gb_jog)

        # --- NÚT SNAP (KHẮC PHỤC LỖI LIMIT) ---
        self.btn_snap = QPushButton("SNAP TARGET TO ACTUAL (RESET)")
        self.btn_snap.setMinimumHeight(45)
        self.btn_snap.setStyleSheet("background-color: #FF9800; color: black; font-weight: bold;")
        self.btn_snap.clicked.connect(self.snap_to_actual)
        layout.addWidget(self.btn_snap)
        
        layout.addStretch()
        self.setLayout(layout)

    def snap_to_actual(self):
        """Buộc Target phải khớp với vị trí thực tế để thoát lỗi kẹt"""
        # Sử dụng đúng tên biến self.main_win.last_fb_pose đã khai báo trong main.py
        self.main_win.target_pos = list(self.main_win.last_fb_pose[:3])
        self.main_win.target_rpy = list(self.main_win.last_fb_pose[3:])
        
        # Cập nhật số trên UI
        self.update_ui_values(self.main_win.target_pos, self.main_win.target_rpy)
        
        # Mở khóa hệ thống
        self.main_win.is_stuck = False
        print("Snap Successful: Target synced with robot feedback.")

    def execute_pose(self):
        if not self.main_win.is_active: return
        if self.main_win.is_stuck:
            QMessageBox.warning(self, "Limit reached", "Please press SNAP before moving.")
            return

        for i, key in enumerate(COORD_KEYS):
            val = self.spins[key].value()
            if i < 3: self.main_win.target_pos[i] = val
            else: self.main_win.target_rpy[i-3] = val
        
        self.main_win.current_mode = MODE_TRAJECTORY
        self.main_win.traj_trigger += 1

    def jog_step(self, key, delta):
        if not self.main_win.is_active or self.main_win.is_stuck: return
        
        idx = COORD_KEYS.index(key)
        if idx < 3: self.main_win.target_pos[idx] += delta
        else: self.main_win.target_rpy[idx-3] += delta
        
        self.update_ui_values(self.main_win.target_pos, self.main_win.target_rpy)
        self.main_win.current_mode = MODE_TRAJECTORY
        self.main_win.traj_trigger += 1

    def update_ui_values(self, pos, rpy):
        self.blockSignals(True)
        for i, key in enumerate(COORD_KEYS):
            val = pos[i] if i < 3 else rpy[i-3]
            self.spins[key].setValue(val)
        self.blockSignals(False)