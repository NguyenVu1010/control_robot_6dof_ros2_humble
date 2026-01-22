import sys
import mmap
import struct
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, QSlider, QPushButton, 
                             QGridLayout, QProgressBar, QMessageBox, QDoubleSpinBox,
                             QTabWidget, QRadioButton, QButtonGroup)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor, QPalette

# --- CẤU HÌNH SHARED MEMORY ---
SHM_NAME = "/robot_control_shm"

# Định nghĩa Mode (Khớp với enum C++)
MODE_IDLE = 0
MODE_POSE = 1
MODE_TRAJECTORY = 2
MODE_JOINT = 3

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Advanced Robot Control Center")
        self.setGeometry(100, 100, 1200, 800)
        self.shm = None
        
        # --- DATA STORAGE ---
        self.current_mode = MODE_POSE
        self.is_active = False
        
        # Mode 1: Pose
        self.target_pos = [0.0]*3
        self.target_rpy = [0.0]*3
        
        # Mode 2: Trajectory (Generator)
        self.traj_t = 0.0
        self.is_traj_running = False
        self.traj_vel_linear = [0.0]*3
        self.traj_vel_angularg = [0.0]*3
        
        # Mode 3: Joint Manual
        self.joint_vel_cmds = [0.0]*6
        
        # Common: Gripper
        self.cmd_gripper = 0.0

        self.init_ui()
        self.connect_shm()

        # Timer 30ms (33Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(30)

    def connect_shm(self):
        try:
            f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(f.fileno(), 0)
            self.status_label.setText("SYSTEM CONNECTED")
            self.status_label.setStyleSheet("color: #00E676; font-weight: bold;")
        except FileNotFoundError:
            self.status_label.setText("CONNECTION FAILED (Launch ROS2 First)")
            self.status_label.setStyleSheet("color: #FF5252; font-weight: bold;")

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # ==========================
        # 1. LEFT PANEL: MONITORING
        # ==========================
        left_panel = QVBoxLayout()
        
        # EE Pose Feedback
        gb_ee = QGroupBox("End-Effector Feedback")
        gb_ee.setStyleSheet("border: 1px solid #555; margin-top:10px;")
        l_ee = QGridLayout()
        self.ee_labels = {}
        tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for i, tag in enumerate(tags):
            l_ee.addWidget(QLabel(f"{tag}:"), i//3, (i%3)*2)
            lbl = QLabel("0.000")
            lbl.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
            lbl.setStyleSheet("color: #FFD740;")
            l_ee.addWidget(lbl, i//3, (i%3)*2 + 1)
            self.ee_labels[tag] = lbl
        gb_ee.setLayout(l_ee)
        left_panel.addWidget(gb_ee)

        # Joint Feedback
        gb_joints = QGroupBox("Joint States")
        l_joints = QGridLayout()
        self.joint_labels = []
        for i in range(6):
            l_joints.addWidget(QLabel(f"J{i+1}"), i, 0)
            pb = QProgressBar()
            pb.setRange(-314, 314)
            pb.setTextVisible(False)
            val = QLabel("0.00")
            l_joints.addWidget(pb, i, 1)
            l_joints.addWidget(val, i, 2)
            self.joint_labels.append((pb, val))
        gb_joints.setLayout(l_joints)
        left_panel.addWidget(gb_joints)

        # ==========================
        # 2. RIGHT PANEL: CONTROL
        # ==========================
        right_panel = QVBoxLayout()

        # --- Master Switch ---
        self.btn_active = QPushButton("ENABLE CONTROL SYSTEM")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(50)
        self.btn_active.setStyleSheet("background-color: #546E7A; color: white; font-weight: bold;")
        self.btn_active.clicked.connect(self.toggle_active)
        right_panel.addWidget(self.btn_active)

        # --- Gripper (Global) ---
        gb_grip = QGroupBox("Gripper Control")
        gb_grip.setStyleSheet("border: 1px solid #FF9800; margin-top:10px; QGroupBox::title {color: #FF9800;}")
        l_grip = QHBoxLayout()
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 80)
        self.grip_slider.setEnabled(False)
        self.grip_slider.valueChanged.connect(self.update_gripper)
        self.grip_val = QLabel("0.00")
        l_grip.addWidget(QLabel("Open"))
        l_grip.addWidget(self.grip_slider)
        l_grip.addWidget(QLabel("Close"))
        l_grip.addWidget(self.grip_val)
        gb_grip.setLayout(l_grip)
        right_panel.addWidget(gb_grip)

        # --- Tabs for Modes ---
        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self.on_tab_change)
        
        # TAB 1: POSE CONTROL
        tab_pose = QWidget()
        l_pose = QGridLayout()
        self.pose_inputs = {}
        for i, tag in enumerate(tags):
            spin = QDoubleSpinBox()
            if i < 3: spin.setRange(-2.0, 2.0); spin.setSingleStep(0.01)
            else: spin.setRange(-3.14, 3.14); spin.setSingleStep(0.05)
            spin.setDecimals(3)
            spin.setEnabled(False)
            spin.valueChanged.connect(self.update_pose_target)
            l_pose.addWidget(QLabel(tag), i, 0)
            l_pose.addWidget(spin, i, 1)
            self.pose_inputs[tag] = spin
        
        btn_sync = QPushButton("Sync Current Pose")
        btn_sync.clicked.connect(self.sync_pose_ui)
        l_pose.addWidget(btn_sync, 6, 0, 1, 2)
        tab_pose.setLayout(l_pose)
        
        # TAB 2: TRAJECTORY GENERATOR
        tab_traj = QWidget()
        l_traj = QVBoxLayout()
        
        self.btn_traj_circle = QPushButton("Start Circle (XY Plane)")
        self.btn_traj_circle.setCheckable(True)
        self.btn_traj_circle.clicked.connect(lambda: self.set_traj_type(0))
        
        self.btn_traj_line = QPushButton("Start Z-Oscillation")
        self.btn_traj_line.setCheckable(True)
        self.btn_traj_line.clicked.connect(lambda: self.set_traj_type(1))
        
        self.lbl_traj_status = QLabel("Status: IDLE")
        self.lbl_traj_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        l_traj.addWidget(QLabel("Auto-generated Velocity Profile:"))
        l_traj.addWidget(self.btn_traj_circle)
        l_traj.addWidget(self.btn_traj_line)
        l_traj.addWidget(self.lbl_traj_status)
        l_traj.addStretch()
        tab_traj.setLayout(l_traj)

        # TAB 3: JOINT MANUAL
        tab_joint = QWidget()
        l_jman = QVBoxLayout()
        self.joint_sliders = []
        for i in range(6):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"J{i+1} Vel:"))
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(-50, 50) # -0.5 to 0.5 rad/s
            s.setValue(0)
            s.valueChanged.connect(self.update_joint_manual)
            s.setEnabled(False)
            h.addWidget(s)
            l_jman.addLayout(h)
            self.joint_sliders.append(s)
        
        btn_stop_j = QPushButton("Stop All Joints")
        btn_stop_j.clicked.connect(self.reset_joint_sliders)
        l_jman.addWidget(btn_stop_j)
        tab_joint.setLayout(l_jman)

        # Add Tabs
        self.tabs.addTab(tab_pose, "Pose Control")
        self.tabs.addTab(tab_traj, "Trajectory")
        self.tabs.addTab(tab_joint, "Joint Manual")

        right_panel.addWidget(self.tabs)

        main_layout.addLayout(left_panel, 4)
        main_layout.addLayout(right_panel, 3)
        
        self.status_label = QLabel("INIT")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        container = QVBoxLayout()
        container.addLayout(main_layout)
        container.addWidget(self.status_label)
        main_widget.setLayout(container)
        self.setCentralWidget(main_widget)

    # --- LOGIC ---

    def toggle_active(self):
        self.is_active = self.btn_active.isChecked()
        if self.is_active:
            self.btn_active.setText("SYSTEM ACTIVE")
            self.btn_active.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.grip_slider.setEnabled(True)
            self.update_ui_state(True)
        else:
            self.btn_active.setText("ENABLE CONTROL SYSTEM")
            self.btn_active.setStyleSheet("background-color: #546E7A; color: white; font-weight: bold;")
            self.grip_slider.setEnabled(False)
            self.update_ui_state(False)
            
            # Send Stop
            if self.shm:
                self.shm.seek(304) # Active Flag
                self.shm.write(struct.pack('?', False))

    def on_tab_change(self, index):
        # 0: Pose, 1: Traj, 2: Joint
        map_mode = [MODE_POSE, MODE_TRAJECTORY, MODE_JOINT]
        self.current_mode = map_mode[index]
        
        # Reset internal states when switching
        self.is_traj_running = False
        self.btn_traj_circle.setChecked(False)
        self.btn_traj_line.setChecked(False)
        self.lbl_traj_status.setText("Status: IDLE")
        self.reset_joint_sliders()
        
        self.update_ui_state(self.is_active)

    def update_ui_state(self, active):
        # Enable/Disable widgets based on Mode
        is_pose = (self.current_mode == MODE_POSE) and active
        is_traj = (self.current_mode == MODE_TRAJECTORY) and active
        is_joint = (self.current_mode == MODE_JOINT) and active

        for s in self.pose_inputs.values(): s.setEnabled(is_pose)
        self.btn_traj_circle.setEnabled(is_traj)
        self.btn_traj_line.setEnabled(is_traj)
        for s in self.joint_sliders: s.setEnabled(is_joint)

    def update_gripper(self, val):
        self.cmd_gripper = val / 100.0
        self.grip_val.setText(f"{self.cmd_gripper:.2f}")

    def update_pose_target(self):
        tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for i, tag in enumerate(tags):
            val = self.pose_inputs[tag].value()
            if i < 3: self.target_pos[i] = val
            else: self.target_rpy[i-3] = val

    def update_joint_manual(self):
        for i, s in enumerate(self.joint_sliders):
            self.joint_vel_cmds[i] = s.value() / 100.0 # scale down

    def reset_joint_sliders(self):
        for s in self.joint_sliders:
            s.blockSignals(True)
            s.setValue(0)
            s.blockSignals(False)
        self.joint_vel_cmds = [0.0]*6

    def set_traj_type(self, type_id):
        if type_id == 0:
            self.is_traj_running = self.btn_traj_circle.isChecked()
            self.btn_traj_line.setChecked(False)
        else:
            self.is_traj_running = self.btn_traj_line.isChecked()
            self.btn_traj_circle.setChecked(False)
            
        self.traj_t = 0.0
        self.lbl_traj_status.setText("RUNNING..." if self.is_traj_running else "IDLE")

    def sync_pose_ui(self):
        # Manually trigger sync (used by button)
        pass 

    def update_loop(self):
        if not self.shm: return

        try:
            # --- READ FEEDBACK (0-144) ---
            self.shm.seek(0)
            data_bytes = self.shm.read(144)
            data = struct.unpack('18d', data_bytes)
            
            # Update GUI
            for i in range(6):
                self.joint_labels[i][0].setValue(int(data[i]*100))
                self.joint_labels[i][1].setText(f"{data[i]:.2f}")
            
            ee_vals = data[12:18]
            tags = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            for i, tag in enumerate(tags):
                self.ee_labels[tag].setText(f"{ee_vals[i]:.3f}")

            # SYNC POSE INPUTS (When not active or not in pose mode)
            if not self.is_active or self.current_mode != MODE_POSE:
                for i, tag in enumerate(tags):
                    self.pose_inputs[tag].blockSignals(True)
                    self.pose_inputs[tag].setValue(ee_vals[i])
                    self.pose_inputs[tag].blockSignals(False)
                    if i < 3: self.target_pos[i] = ee_vals[i]
                    else: self.target_rpy[i-3] = ee_vals[i]

            # --- TRAJECTORY GENERATION (Python Side) ---
            if self.is_active and self.current_mode == MODE_TRAJECTORY and self.is_traj_running:
                self.traj_t += 0.03 # dt = 30ms
                
                if self.btn_traj_circle.isChecked():
                    # Circle radius 0.1m, Period 5s
                    w = 2 * 3.14159 / 5.0
                    vx = -0.1 * w * math.sin(w * self.traj_t)
                    vy = 0.1 * w * math.cos(w * self.traj_t)
                    self.traj_vel_linear = [vx, vy, 0.0]
                    self.traj_vel_angularg = [0.0, 0.0, 0.0]
                
                elif self.btn_traj_line.isChecked():
                    # Z oscillation
                    vz = 0.05 * math.sin(2.0 * self.traj_t)
                    self.traj_vel_linear = [0.0, 0.0, vz]
                    self.traj_vel_angularg = [0.0, 0.0, 0.0]
            else:
                self.traj_vel_linear = [0.0]*3
                self.traj_vel_angularg = [0.0]*3

            # --- WRITE COMMANDS ---
            if self.is_active:
                # 1. Mode (Offset 144)
                self.shm.seek(144)
                self.shm.write(struct.pack('i', self.current_mode))
                
                # 2. Target Pose (Offset 152 - Aligned 8 bytes)
                self.shm.seek(152)
                self.shm.write(struct.pack('3d', *self.target_pos))
                
                # 3. Target RPY (Offset 176)
                self.shm.seek(176)
                self.shm.write(struct.pack('3d', *self.target_rpy))
                
                # 4. Traj Vel (Offset 200)
                self.shm.seek(200)
                self.shm.write(struct.pack('3d', *self.traj_vel_linear))
                self.shm.seek(224)
                self.shm.write(struct.pack('3d', *self.traj_vel_angularg))
                
                # 5. Manual Joints (Offset 248)
                self.shm.seek(248)
                self.shm.write(struct.pack('6d', *self.joint_vel_cmds))
                
                # 6. Gripper (Offset 296)
                self.shm.seek(296)
                self.shm.write(struct.pack('d', self.cmd_gripper))
                
                # 7. Active Flag (Offset 304)
                self.shm.seek(304)
                self.shm.write(struct.pack('?', True))

        except Exception as e:
            # print(f"SHM Error: {e}")
            pass

    def closeEvent(self, event):
        if self.shm: 
            self.shm.seek(304)
            self.shm.write(struct.pack('?', False))
            self.shm.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Dark Mode
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(40, 40, 40))
    p.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    p.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.Button, QColor(60, 60, 60))
    p.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    p.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    p.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(p)

    win = RobotInterface()
    win.show()
    sys.exit(app.exec())