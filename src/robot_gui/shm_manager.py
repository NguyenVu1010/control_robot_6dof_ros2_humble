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
        if not self.shm: return None
        try:
            self.shm.seek(OFF_READ_START)
            data = struct.unpack('18d', self.shm.read(144))
            return {
                'joints': data[0:6],
                'pose': data[12:18]
            }
        except: return None

    def write_command(self, active, mode, target_pos, target_rpy, traj_dur, traj_trig, manual_vel, gripper):
        if not self.shm: return
        try:
            self.shm.seek(OFF_MODE); self.shm.write(struct.pack('i', mode))
            self.shm.seek(OFF_TARGET_POS); self.shm.write(struct.pack('3d', *target_pos))
            self.shm.seek(OFF_TARGET_RPY); self.shm.write(struct.pack('3d', *target_rpy))
            self.shm.seek(OFF_TRAJ_DUR); self.shm.write(struct.pack('d', traj_dur))
            self.shm.seek(OFF_TRAJ_TRIG); self.shm.write(struct.pack('i', traj_trig))
            self.shm.seek(OFF_MANUAL_J); self.shm.write(struct.pack('6d', *manual_vel))
            self.shm.seek(OFF_GRIPPER); self.shm.write(struct.pack('d', gripper))
            self.shm.seek(OFF_ACTIVE); self.shm.write(struct.pack('?', active))
        except: pass