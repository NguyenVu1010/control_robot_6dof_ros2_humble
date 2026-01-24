from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton
from PyQt6.QtCore import Qt

class JointTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        self.sliders = []
        for i in range(6):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"J{i+1}:"))
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(-100, 100)
            s.valueChanged.connect(self.on_move)
            h.addWidget(s)
            self.sliders.append(s)
            layout.addLayout(h)
        
        btn = QPushButton("Reset All")
        btn.clicked.connect(self.reset)
        layout.addWidget(btn)
        self.setLayout(layout)

    def on_move(self):
        for i, s in enumerate(self.sliders):
            self.main_win.manual_vel[i] = s.value() / 100.0

    def reset(self):
        for s in self.sliders: s.setValue(0)