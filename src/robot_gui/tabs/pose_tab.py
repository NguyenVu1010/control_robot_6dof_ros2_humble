from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QLabel, QDoubleSpinBox, QPushButton
from config import COORD_KEYS

class PoseTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout(self)
        
        # --- Group nhập liệu ---
        gb = QGroupBox("Target Cartesian Pose")
        grid = QGridLayout()
        self.spins = {}
        
        for i, key in enumerate(COORD_KEYS):
            grid.addWidget(QLabel(key + ":"), i, 0)
            s = QDoubleSpinBox()
            s.setRange(-5.0, 5.0)
            s.setDecimals(3)
            s.setSingleStep(0.005)
            grid.addWidget(s, i, 1)
            self.spins[key] = s
            
        # Nút GO: Chỉ khi nhấn nút này lệnh Pose mới thực thi
        self.btn_go = QPushButton("GO / EXECUTE")
        self.btn_go.setMinimumHeight(45)
        self.btn_go.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
        self.btn_go.clicked.connect(self.on_go_clicked)
        grid.addWidget(self.btn_go, len(COORD_KEYS), 0, 1, 2)
        
        gb.setLayout(grid)
        layout.addWidget(gb)
        
        # Nút Snap: Lấy vị trí hiện tại nạp vào SpinBox
        self.btn_snap = QPushButton("SNAP FROM ACTUAL")
        self.btn_snap.clicked.connect(self.snap_actual)
        layout.addWidget(self.btn_snap)
        
        layout.addStretch()

    def get_pos(self):
        """Trả về [X, Y, Z] từ SpinBoxes"""
        return [self.spins['X'].value(), self.spins['Y'].value(), self.spins['Z'].value()]

    def get_rpy(self):
        """Trả về [Roll, Pitch, Yaw] từ SpinBoxes"""
        return [self.spins['Roll'].value(), self.spins['Pitch'].value(), self.spins['Yaw'].value()]

    def on_go_clicked(self):
        """Kích hoạt di chuyển Pose"""
        if not self.main_win.is_active: return
        # Nạp giá trị từ UI vào biến mục tiêu của hệ thống
        self.main_win.target_pos = self.get_pos()
        self.main_win.target_rpy = self.get_rpy()
        # Kích hoạt trigger
        self.main_win.traj_trigger += 1

    def snap_actual(self):
        """Lấy feedback nạp vào SpinBox"""
        if hasattr(self.main_win, 'last_fb_pose'):
            self.update_ui_values(self.main_win.last_fb_pose[:3], self.main_win.last_fb_pose[3:])

    def update_ui_values(self, pos, rpy):
        """Cập nhật số hiển thị trên SpinBoxes (Dùng blockSignals để tránh nhiễu)"""
        for i, key in enumerate(COORD_KEYS):
            self.spins[key].blockSignals(True)
            val = pos[i] if i < 3 else rpy[i-3]
            self.spins[key].setValue(val)
            self.spins[key].blockSignals(False)