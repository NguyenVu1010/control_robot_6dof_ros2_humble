SHM_NAME = "/robot_control_shm"

# OFFSETS PHẢN HỒI
OFF_JOINT_POS   = 0
OFF_JOINT_VEL   = 48
OFF_EE_POS      = 96
OFF_EE_RPY      = 120

# OFFSETS LỆNH
OFF_MODE        = 144
OFF_TARGET_POS  = 152
OFF_TARGET_RPY  = 176
OFF_TRAJ_VEL    = 200  # 6 doubles (vx, vy, vz, wx, wy, wz)
OFF_TRAJ_DUR    = 248
OFF_TRAJ_TRIG   = 256
OFF_MANUAL_J    = 264  # 6 doubles (j1...j6 vel)
OFF_GRIPPER     = 312
OFF_ACTIVE      = 320

# Modes
MODE_IDLE = 0
MODE_POSE = 1
MODE_TRAJECTORY = 2
MODE_JOINT = 3

COORD_KEYS = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
JOINT_KEYS = ["J1", "J2", "J3", "J4", "J5", "J6"]
MAX_ALLOWED_ERROR = 0.05