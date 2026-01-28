from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton
from PyQt6.QtCore import Qt
from config import MODE_JOINT

class JointTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        
        layout.addWidget(QLabel("Manual Joint Velocity Control (Jogging)"))
        
        self.sliders = []
        for i in range(6):
            h = QHBoxLayout()
            lbl = QLabel(f"J{i+1}: 0.00")
            lbl.setFixedWidth(60)
            
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(-100, 100)
            s.setValue(0)
            # Truyền thêm label để cập nhật số hiển thị
            s.valueChanged.connect(lambda v, l=lbl, idx=i: self.on_slider_change(v, l, idx))
            
            h.addWidget(QLabel(f"J{i+1}"))
            h.addWidget(s)
            h.addWidget(lbl)
            self.sliders.append(s)
            layout.addLayout(h)
        
        self.btn_stop = QPushButton("STOP ALL JOINTS")
        self.btn_stop.clicked.connect(self.reset_all)
        self.btn_stop.setStyleSheet("background-color: #C62828; color: white;")
        layout.addWidget(self.btn_stop)
        
        self.setLayout(layout)

    def on_slider_change(self, value, label, idx):
        vel = value / 100.0
        label.setText(f"{vel:.2f}")
        if self.main_win.is_active:
            self.main_win.manual_vel[idx] = vel
            # Ép mode về Joint khi di chuyển slider
            self.main_win.current_mode = MODE_JOINT 

    def reset_all(self):
        for s in self.sliders:
            s.setValue(0)
        self.main_win.manual_vel = [0.0]*6