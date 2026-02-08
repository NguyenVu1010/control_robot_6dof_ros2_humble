import posix_ipc
import mmap
import struct
from config import *

class SHMManager:
    def __init__(self):
        self.shm = None
        self.map = None

    def connect(self):
        try:
            self.shm = posix_ipc.SharedMemory(SHM_NAME)
            self.map = mmap.mmap(self.shm.fd, self.shm.size)
            return True
        except: return False

    def read_feedback(self):
        if not self.map: return None
        try:
            self.map.seek(0)
            # Đọc 18 số double (6 pos, 6 vel, 3 ee_pos, 3 ee_rpy)
            d = struct.unpack("18d", self.map.read(144))
            
            return {
                'joints': d[0:6],      # Đổi 'joint_pos' thành 'joints' để hết lỗi KeyError
                'joint_vel': d[6:12],
                'ee_pos': d[12:15],
                'ee_rpy': d[15:18],
                'pose': d[12:18]       # Bao gồm cả ee_pos và ee_rpy để main.py dùng
            }
        except Exception as e:
            print(f"Error reading SHM: {e}")
            return None

    def write_command(self, active, mode, target_p, target_r, dur, trig, manual_j, gripper, ee_vel=[0,0,0,0,0,0]):
        if not self.map: return
        
        # Mode
        self.map.seek(OFF_MODE)
        self.map.write(struct.pack("i", int(mode)))

        # Target Pose
        self.map.seek(OFF_TARGET_POS)
        self.map.write(struct.pack("3d", *target_p))
        self.map.seek(OFF_TARGET_RPY)
        self.map.write(struct.pack("3d", *target_r))

        # EE Velocity (Offset 200 trong config của bạn)
        self.map.seek(OFF_TRAJ_VEL)
        self.map.write(struct.pack("6d", *ee_vel))

        # Trigger & Duration
        self.map.seek(OFF_TRAJ_DUR)
        self.map.write(struct.pack("d", float(dur)))
        self.map.seek(OFF_TRAJ_TRIG)
        self.map.write(struct.pack("i", int(trig)))

        # Manual Joint
        self.map.seek(OFF_MANUAL_J)
        self.map.write(struct.pack("6d", *manual_j))

        # Gripper & Active
        self.map.seek(OFF_GRIPPER)
        self.map.write(struct.pack("d", float(gripper)))
        self.map.seek(OFF_ACTIVE)
        self.map.write(struct.pack("??", bool(active), True))