import sys
import mmap
import struct
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, QSlider, QPushButton, 
                             QGridLayout, QProgressBar, QMessageBox, QDoubleSpinBox,
                             QTabWidget)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor, QPalette

# --- CẤU HÌNH SHARED MEMORY ---
SHM_NAME = "/robot_control_shm"

# Offset Config
OFF_READ_START  = 0
OFF_MODE        = 144
OFF_TARGET_POS  = 152
OFF_TARGET_RPY  = 176
OFF_TRAJ_DUR    = 200
OFF_TRAJ_TRIG   = 208
OFF_MANUAL_J    = 216
OFF_GRIPPER     = 264
OFF_ACTIVE      = 272

# Modes
MODE_IDLE = 0
MODE_POSE = 1
MODE_TRAJECTORY = 2
MODE_JOINT = 3

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Center (Full Version)")
        self.setGeometry(100, 100, 1100, 850)
        self.shm = None
        
        # Data Storage
        self.current_mode = MODE_POSE
        self.is_active = False
        
        self.target_pos = [0.0]*3
        self.target_rpy = [0.0]*3
        self.traj_duration = 5.0
        self.traj_trigger = 0
        self.manual_vel = [0.0]*6
        self.cmd_gripper = 0.0

        # Keys
        self.coord_keys = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        self.init_ui()
        self.connect_shm()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(30)

    def connect_shm(self):
        try:
            f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(f.fileno(), 0)
            self.status_label.setText("CONNECTED")
            self.status_label.setStyleSheet("color: #00E676; font-weight: bold;")
        except FileNotFoundError:
            self.status_label.setText("DISCONNECTED")
            self.status_label.setStyleSheet("color: #FF5252; font-weight: bold;")

    def init_ui(self):
        main_widget = QWidget()
        layout = QHBoxLayout()
        
        # --- LEFT PANEL: MONITOR ---
        left_panel = QVBoxLayout()
        
        # EE Feedback
        gb_ee = QGroupBox("Feedback: End-Effector")
        gb_ee.setStyleSheet("border: 1px solid #555;")
        l_ee = QGridLayout()
        self.ee_labels = {}
        for i, tag in enumerate(self.coord_keys):
            l_ee.addWidget(QLabel(f"{tag}:"), i//3, (i%3)*2)
            lbl = QLabel("0.000")
            lbl.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
            lbl.setStyleSheet("color: #FFD740;")
            l_ee.addWidget(lbl, i//3, (i%3)*2 + 1)
            self.ee_labels[tag] = lbl
        gb_ee.setLayout(l_ee)
        left_panel.addWidget(gb_ee)

        # Joint Feedback
        gb_joints = QGroupBox("Feedback: Joints")
        l_joints = QGridLayout()
        self.joint_labels = []
        for i in range(6):
            l_joints.addWidget(QLabel(f"J{i+1}:"), i, 0)
            pb = QProgressBar()
            pb.setRange(-314, 314)
            pb.setTextVisible(False)
            val = QLabel("0.00")
            val.setFixedWidth(50)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            l_joints.addWidget(pb, i, 1)
            l_joints.addWidget(val, i, 2)
            self.joint_labels.append((pb, val))
        gb_joints.setLayout(l_joints)
        left_panel.addWidget(gb_joints)

        # --- RIGHT PANEL: CONTROL ---
        right_panel = QVBoxLayout()

        # Switch
        self.btn_active = QPushButton("ENABLE CONTROL")
        self.btn_active.setCheckable(True)
        self.btn_active.setMinimumHeight(50)
        self.btn_active.setStyleSheet("background-color: #546E7A; color: white; font-weight: bold;")
        self.btn_active.clicked.connect(self.toggle_active)
        right_panel.addWidget(self.btn_active)

        # Gripper
        gb_grip = QGroupBox("Gripper")
        gb_grip.setStyleSheet("border: 1px solid #FF9800; QGroupBox::title {color: #FF9800;}")
        l_grip = QHBoxLayout()
        self.grip_slider = QSlider(Qt.Orientation.Horizontal)
        self.grip_slider.setRange(0, 80)
        self.grip_slider.setEnabled(False)
        self.grip_slider.valueChanged.connect(self.update_gripper)
        l_grip.addWidget(QLabel("Open"))
        l_grip.addWidget(self.grip_slider)
        l_grip.addWidget(QLabel("Close"))
        gb_grip.setLayout(l_grip)
        right_panel.addWidget(gb_grip)

        # TABS
        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self.on_tab_change)

        # ------------------------------------
        # TAB 1: POSE CONTROL (P-Controller)
        # ------------------------------------
        tab_pose = QWidget()
        l_pose = QVBoxLayout()
        
        gb_pose_in = QGroupBox("Target Coordinates (Direct)")
        gl_pose = QGridLayout()
        self.pose_inputs = {}
        for i, tag in enumerate(self.coord_keys):
            spin = self.create_spinbox()
            spin.valueChanged.connect(self.update_target_from_pose_tab)
            gl_pose.addWidget(QLabel(tag), i, 0)
            gl_pose.addWidget(spin, i, 1)
            self.pose_inputs[tag] = spin
        gb_pose_in.setLayout(gl_pose)
        
        btn_sync_pose = QPushButton("Sync Current Pose")
        btn_sync_pose.clicked.connect(self.sync_pose_ui)
        
        l_pose.addWidget(gb_pose_in)
        l_pose.addWidget(btn_sync_pose)
        l_pose.addStretch()
        tab_pose.setLayout(l_pose)

        # ------------------------------------
        # TAB 2: TRAJECTORY (Point-to-Point)
        # ------------------------------------
        tab_traj = QWidget()
        l_traj = QVBoxLayout()
        
        # Input Destination
        gb_dest = QGroupBox("Set Destination Point")
        gl_dest = QGridLayout()
        self.traj_inputs = {} # Separate inputs for trajectory tab
        for i, tag in enumerate(self.coord_keys):
            spin = self.create_spinbox()
            # Khi đổi giá trị ở đây, cũng cập nhật vào biến target
            spin.valueChanged.connect(self.update_target_from_traj_tab)
            gl_dest.addWidget(QLabel(tag), i, 0)
            gl_dest.addWidget(spin, i, 1)
            self.traj_inputs[tag] = spin
        gb_dest.setLayout(gl_dest)
        
        btn_sync_traj = QPushButton("Set Destination = Current (Reset)")
        btn_sync_traj.clicked.connect(self.sync_pose_ui)

        # Duration
        h_dur = QHBoxLayout()
        h_dur.addWidget(QLabel("Time Duration (s):"))
        self.spin_dur = QDoubleSpinBox()
        self.spin_dur.setRange(0.5, 60.0)
        self.spin_dur.setValue(5.0)
        self.spin_dur.valueChanged.connect(self.update_traj_data)
        h_dur.addWidget(self.spin_dur)

        # Execute Button
        self.btn_exec_traj = QPushButton("EXECUTE TRAJECTORY (GO)")
        self.btn_exec_traj.setMinimumHeight(50)
        self.btn_exec_traj.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;")
        self.btn_exec_traj.setEnabled(False)
        self.btn_exec_traj.clicked.connect(self.trigger_trajectory)

        l_traj.addWidget(gb_dest)
        l_traj.addWidget(btn_sync_traj)
        l_traj.addLayout(h_dur)
        l_traj.addWidget(self.btn_exec_traj)
        l_traj.addStretch()
        tab_traj.setLayout(l_traj)

        # ------------------------------------
        # TAB 3: JOINT MANUAL
        # ------------------------------------
        tab_joint = QWidget()
        l_jman = QVBoxLayout()
        self.joint_sliders = []
        for i in range(6):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"J{i+1}:"))
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(-50, 50)
            s.setValue(0)
            s.setEnabled(False)
            s.valueChanged.connect(self.update_manual_joints)
            h.addWidget(s)
            self.joint_sliders.append(s)
            l_jman.addLayout(h)
        
        btn_stop_j = QPushButton("STOP JOINTS")
        btn_stop_j.setStyleSheet("background-color: #F44336; color: white;")
        btn_stop_j.clicked.connect(self.reset_joint_sliders)
        l_jman.addWidget(btn_stop_j)
        tab_joint.setLayout(l_jman)

        self.tabs.addTab(tab_pose, "Pose (Hold)")
        self.tabs.addTab(tab_traj, "Trajectory (P2P)")
        self.tabs.addTab(tab_joint, "Joint Manual")
        
        right_panel.addWidget(self.tabs)

        layout.addLayout(left_panel, 4)
        layout.addLayout(right_panel, 3)
        
        self.status_label = QLabel("INIT")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        container = QVBoxLayout()
        container.addLayout(layout)
        container.addWidget(self.status_label)
        main_widget.setLayout(container)
        self.setCentralWidget(main_widget)

    # --- HELPERS ---
    def create_spinbox(self):
        spin = QDoubleSpinBox()
        spin.setRange(-5.0, 5.0) # XYZ limits
        spin.setSingleStep(0.01)
        spin.setDecimals(3)
        spin.setEnabled(False)
        spin.setStyleSheet("padding: 5px; font-size: 14px;")
        return spin

    # --- LOGIC ---

    def toggle_active(self):
        self.is_active = self.btn_active.isChecked()
        if self.is_active:
            self.btn_active.setText("SYSTEM ACTIVE")
            self.btn_active.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.grip_slider.setEnabled(True)
            self.update_ui_state(True)
        else:
            self.btn_active.setText("ENABLE CONTROL")
            self.btn_active.setStyleSheet("background-color: #546E7A; color: white; font-weight: bold;")
            self.grip_slider.setEnabled(False)
            self.update_ui_state(False)
            self.reset_joint_sliders()
            if self.shm:
                self.shm.seek(OFF_ACTIVE)
                self.shm.write(struct.pack('?', False))

    def on_tab_change(self, index):
        modes = [MODE_POSE, MODE_TRAJECTORY, MODE_JOINT]
        self.current_mode = modes[index]
        self.update_ui_state(self.is_active)
        if index != 2: self.reset_joint_sliders()

    def update_ui_state(self, active):
        # Enable Pose inputs only in Pose Mode
        is_pose = (self.current_mode == MODE_POSE) and active
        for s in self.pose_inputs.values(): s.setEnabled(is_pose)
        
        # Enable Traj inputs only in Traj Mode
        is_traj = (self.current_mode == MODE_TRAJECTORY) and active
        for s in self.traj_inputs.values(): s.setEnabled(is_traj)
        self.btn_exec_traj.setEnabled(is_traj)
        
        # Manual Joints
        is_joint = (self.current_mode == MODE_JOINT) and active
        for s in self.joint_sliders: s.setEnabled(is_joint)

    def update_gripper(self, val):
        self.cmd_gripper = val / 100.0

    # Cập nhật biến target từ Tab Pose
    def update_target_from_pose_tab(self):
        for i, key in enumerate(self.coord_keys):
            val = self.pose_inputs[key].value()
            if i < 3: self.target_pos[i] = val
            else: self.target_rpy[i-3] = val
    
    # Cập nhật biến target từ Tab Trajectory
    def update_target_from_traj_tab(self):
        for i, key in enumerate(self.coord_keys):
            val = self.traj_inputs[key].value()
            if i < 3: self.target_pos[i] = val
            else: self.target_rpy[i-3] = val

    def update_traj_data(self):
        self.traj_duration = self.spin_dur.value()

    def trigger_trajectory(self):
        self.traj_trigger += 1 # Gửi trigger xuống C++

    def update_manual_joints(self):
        for i, s in enumerate(self.joint_sliders):
            self.manual_vel[i] = s.value() / 100.0

    def reset_joint_sliders(self):
        for s in self.joint_sliders:
            s.blockSignals(True)
            s.setValue(0)
            s.blockSignals(False)
        self.manual_vel = [0.0]*6

    def sync_pose_ui(self):
        # Logic này được xử lý trong update_loop khi inactive
        pass

    def update_loop(self):
        if not self.shm: return

        try:
            # 1. READ
            self.shm.seek(OFF_READ_START)
            data_bytes = self.shm.read(144) 
            data = struct.unpack('18d', data_bytes)
            
            # Update Feedback UI
            for i in range(6):
                self.joint_labels[i][0].setValue(int(data[i]*100))
                self.joint_labels[i][1].setText(f"{data[i]:.2f}")
            
            ee_vals = data[12:18]
            for i, key in enumerate(self.coord_keys):
                self.ee_labels[key].setText(f"{ee_vals[i]:.3f}")

            # 2. SYNC: Khi không Active, điền giá trị hiện tại vào cả 2 tab
            # Để khi user chuyển tab nào thì ô nhập liệu cũng đúng vị trí thật
            if not self.is_active:
                for i, key in enumerate(self.coord_keys):
                    val = ee_vals[i]
                    
                    # Update Pose Tab
                    self.pose_inputs[key].blockSignals(True)
                    self.pose_inputs[key].setValue(val)
                    self.pose_inputs[key].blockSignals(False)
                    
                    # Update Traj Tab
                    self.traj_inputs[key].blockSignals(True)
                    self.traj_inputs[key].setValue(val)
                    self.traj_inputs[key].blockSignals(False)
                    
                    # Internal Variable Sync
                    if i < 3: self.target_pos[i] = val
                    else: self.target_rpy[i-3] = val

            # 3. WRITE
            if self.is_active:
                self.shm.seek(OFF_MODE)
                self.shm.write(struct.pack('i', self.current_mode))
                
                self.shm.seek(OFF_TARGET_POS)
                self.shm.write(struct.pack('3d', *self.target_pos))
                self.shm.seek(OFF_TARGET_RPY)
                self.shm.write(struct.pack('3d', *self.target_rpy))
                
                self.shm.seek(OFF_TRAJ_DUR)
                self.shm.write(struct.pack('d', self.traj_duration))
                self.shm.seek(OFF_TRAJ_TRIG)
                self.shm.write(struct.pack('i', self.traj_trigger))
                
                self.shm.seek(OFF_MANUAL_J)
                self.shm.write(struct.pack('6d', *self.manual_vel))
                
                self.shm.seek(OFF_GRIPPER)
                self.shm.write(struct.pack('d', self.cmd_gripper))
                
                self.shm.seek(OFF_ACTIVE)
                self.shm.write(struct.pack('?', True))

        except Exception as e:
            pass

    def closeEvent(self, event):
        if self.shm: 
            self.shm.seek(OFF_ACTIVE)
            self.shm.write(struct.pack('?', False))
            self.shm.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
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
    app.setPalette(p)

    win = RobotInterface()
    win.show()
    sys.exit(app.exec())