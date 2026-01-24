from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QGridLayout, QLabel, QProgressBar
from config import COORD_KEYS

class MonitorPanel(QGroupBox):
    def __init__(self):
        super().__init__("Robot Monitor (Feedback)")
        layout = QVBoxLayout()
        
        # Pose Display
        pose_gb = QGroupBox("End-Effector Pose")
        self.pose_grid = QGridLayout()
        self.pose_labels = {}
        for i, key in enumerate(COORD_KEYS):
            self.pose_grid.addWidget(QLabel(key + ":"), i//3, (i%3)*2)
            lbl = QLabel("0.000")
            lbl.setStyleSheet("color: #FFD740; font-weight: bold;")
            self.pose_grid.addWidget(lbl, i//3, (i%3)*2 + 1)
            self.pose_labels[key] = lbl
        pose_gb.setLayout(self.pose_grid)
        layout.addWidget(pose_gb)

        # Joint Display
        joint_gb = QGroupBox("Joint Positions")
        joint_layout = QGridLayout()
        self.joint_widgets = []
        for i in range(6):
            joint_layout.addWidget(QLabel(f"J{i+1}:"), i, 0)
            pb = QProgressBar()
            pb.setRange(-314, 314)
            pb.setTextVisible(False)
            val_lbl = QLabel("0.00")
            joint_layout.addWidget(pb, i, 1)
            joint_layout.addWidget(val_lbl, i, 2)
            self.joint_widgets.append((pb, val_lbl))
        joint_gb.setLayout(joint_layout)
        layout.addWidget(joint_gb)
        
        self.setLayout(layout)

    def update_display(self, data):
        for i, key in enumerate(COORD_KEYS):
            self.pose_labels[key].setText(f"{data['pose'][i]:.3f}")
        for i in range(6):
            val = data['joints'][i]
            self.joint_widgets[i][0].setValue(int(val * 100))
            self.joint_widgets[i][1].setText(f"{val:.2f}")