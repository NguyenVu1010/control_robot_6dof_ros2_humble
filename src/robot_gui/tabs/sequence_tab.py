from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QListWidget, QFileDialog, QLabel
from config import MODE_TRAJECTORY

class SequenceTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        
        self.list_widget = QListWidget()
        layout.addWidget(QLabel("Step List:"))
        layout.addWidget(self.list_widget)
        
        self.btn_add = QPushButton("Add Current Step")
        self.btn_add.clicked.connect(self.add)
        layout.addWidget(self.btn_add)
        
        self.btn_run = QPushButton("RUN SEQUENCE")
        self.btn_run.setStyleSheet("background-color: #D32F2F; color: white; font-weight: bold;")
        self.btn_run.clicked.connect(self.toggle_run)
        layout.addWidget(self.btn_run)
        
        self.setLayout(layout)

    def add(self):
        self.main_win.seq_manager.add_step(
            self.main_win.target_pos, self.main_win.target_rpy, 
            self.main_win.traj_duration, self.main_win.cmd_gripper
        )
        self.list_widget.addItem(f"Step {len(self.main_win.seq_manager.steps)}")

    def toggle_run(self):
        mgr = self.main_win.seq_manager
        if not mgr.is_running:
            if not mgr.steps: return
            mgr.is_running = True
            mgr.current_idx = 0
            mgr.timer_count = 0
            self.main_win.current_mode = MODE_TRAJECTORY
            self.btn_run.setText("STOP")
        else:
            mgr.is_running = False
            self.btn_run.setText("RUN SEQUENCE")