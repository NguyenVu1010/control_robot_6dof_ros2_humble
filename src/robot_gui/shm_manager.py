import mmap
import struct
from config import *

class SHMManager:
    def __init__(self):
        self.shm = None

    def connect(self):
        try:
            f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(f.fileno(), 0)
            return True
        except FileNotFoundError:
            return False

    def read_feedback(self):
        try:
            self.shm.seek(0)
            data_bytes = self.shm.read(144) # Đọc đúng 144 bytes
            data = struct.unpack(FB_STRUCT_FORMAT, data_bytes)
            return {
                'joints': data[0:6],
                'pose': data[12:18] # X, Y, Z, R, P, Y
            }
        except:
            return None

    def write_command(self, active, mode, pos, rpy, dur, trig, manual_vel, gripper):
        # Ghi Mode
        self.shm.seek(OFF_MODE)
        self.shm.write(struct.pack('i', mode))
        
        # Ghi Target
        self.shm.seek(OFF_TARGET_POS)
        self.shm.write(struct.pack('3d', *pos))
        self.shm.seek(OFF_TARGET_RPY)
        self.shm.write(struct.pack('3d', *rpy))
        
        # Ghi Traj Duration & Trigger
        self.shm.seek(OFF_TRAJ_DUR)
        self.shm.write(struct.pack('d', dur))
        self.shm.seek(OFF_TRAJ_TRIG)
        self.shm.write(struct.pack('i', trig))
        
        # Ghi Manual Velocity
        self.shm.seek(OFF_MANUAL_J)
        self.shm.write(struct.pack('6d', *manual_vel))
        
        # Ghi Gripper
        self.shm.seek(OFF_GRIPPER)
        self.shm.write(struct.pack('d', gripper))
        
        # Ghi Active Flag
        self.shm.seek(OFF_ACTIVE)
        self.shm.write(struct.pack('?', active))