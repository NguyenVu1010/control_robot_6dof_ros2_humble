# --- SHARED MEMORY CONFIG ---
SHM_NAME = "/robot_control_shm"

# OFFSETS PHẢN HỒI (READ)
OFF_JOINT_POS   = 0
OFF_JOINT_VEL   = 48
OFF_EE_POS      = 96
OFF_EE_RPY      = 120

# OFFSETS LỆNH (WRITE)
OFF_MODE        = 144
OFF_TARGET_POS  = 152
OFF_TARGET_RPY  = 176
OFF_TRAJ_VEL    = 200  # Gồm 6 double (linear + angular)
OFF_TRAJ_DUR    = 248
OFF_TRAJ_TRIG   = 256
OFF_MANUAL_J    = 264
OFF_GRIPPER     = 312
OFF_ACTIVE      = 320

# Định dạng dữ liệu cho module shm_manager.py dùng struct.unpack
# Feedback: 6d (pos) + 6d (vel) + 3d (ee_pos) + 3d (ee_rpy) = 18 doubles
FB_STRUCT_FORMAT = "18d" 

# Modes
MODE_IDLE = 0
MODE_POSE = 1
MODE_TRAJECTORY = 2
MODE_JOINT = 3

COORD_KEYS = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
MAX_ALLOWED_ERROR = 0.05