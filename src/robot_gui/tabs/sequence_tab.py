from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QListWidget, QGridLayout, QLabel, QFileDialog, QMessageBox
from config import MODE_TRAJECTORY

class SequenceTab(QWidget):
    def __init__(self, main_win):
        super().__init__()
        self.main_win = main_win
        layout = QVBoxLayout()
        
        # Danh sách hiển thị
        self.list_widget = QListWidget()
        layout.addWidget(QLabel("Sequence Steps:"))
        layout.addWidget(self.list_widget)
        
        # Các nút chức năng
        grid = QGridLayout()
        self.btn_add = QPushButton("Add Current State")
        self.btn_add.clicked.connect(self.add_current_state)
        
        self.btn_remove = QPushButton("Delete Selected")
        self.btn_remove.clicked.connect(self.remove_step)
        
        self.btn_save = QPushButton("Save to JSON")
        self.btn_save.clicked.connect(self.save_sequence)
        
        self.btn_load = QPushButton("Load from JSON")
        self.btn_load.clicked.connect(self.load_sequence)

        grid.addWidget(self.btn_add, 0, 0); grid.addWidget(self.btn_remove, 0, 1)
        grid.addWidget(self.btn_save, 1, 0); grid.addWidget(self.btn_load, 1, 1)
        layout.addLayout(grid)
        
        # Nút Chạy chuỗi
        self.btn_run = QPushButton("RUN SEQUENCE")
        self.btn_run.setMinimumHeight(50)
        self.btn_run.setStyleSheet("background-color: #D32F2F; color: white; font-weight: bold;")
        self.btn_run.clicked.connect(self.toggle_run)
        layout.addWidget(self.btn_run)
        
        self.setLayout(layout)

    def update_list_ui(self):
        """Cập nhật lại danh sách hiển thị từ dữ liệu trong Manager"""
        self.list_widget.clear()
        for i, step in enumerate(self.main_win.seq_manager.steps):
            txt = f"[{i+1}] P:{step['pos']} | G:{step['grip']:.2f} | T:{step['dur']}s"
            self.list_widget.addItem(txt)

    def add_current_state(self):
        # Lấy dữ liệu hiện tại từ main_win
        self.main_win.seq_manager.add_step(
            self.main_win.target_pos,
            self.main_win.target_rpy,
            self.main_win.traj_duration,
            self.main_win.cmd_gripper
        )
        self.update_list_ui()

    def remove_step(self):
        row = self.list_widget.currentRow()
        if row >= 0:
            self.main_win.seq_manager.steps.pop(row)
            self.update_list_ui()

    def save_sequence(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Sequence", "", "JSON Files (*.json)")
        if path:
            if self.main_win.seq_manager.save_to_json(path):
                QMessageBox.information(self, "Success", "Sequence saved successfully!")

    def load_sequence(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Sequence", "", "JSON Files (*.json)")
        if path:
            if self.main_win.seq_manager.load_from_json(path):
                self.update_list_ui()
                QMessageBox.information(self, "Success", "Sequence loaded successfully!")

    def toggle_run(self):
        mgr = self.main_win.seq_manager
        if not mgr.is_running:
            if not mgr.steps:
                QMessageBox.warning(self, "Warning", "Sequence is empty!")
                return
            mgr.is_running = True
            mgr.current_idx = 0
            mgr.timer_count = 0
            self.main_win.current_mode = MODE_TRAJECTORY
            self.btn_run.setText("STOP SEQUENCE")
            self.btn_run.setStyleSheet("background-color: #FF9800; color: white;")
        else:
            mgr.is_running = False
            self.btn_run.setText("RUN SEQUENCE")
            self.btn_run.setStyleSheet("background-color: #D32F2F; color: white;")