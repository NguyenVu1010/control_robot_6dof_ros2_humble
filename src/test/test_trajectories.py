import mmap
import struct
import time
import math
import sys

# --- IMPORT CONFIG ---
SHM_NAME = "/robot_control_shm"
OFF_MODE = 144
OFF_TARGET_POS = 152
OFF_TARGET_RPY = 176
OFF_TRAJ_VEL_LIN = 200  # [vx, vy, vz, wx, wy, wz] chiếm 48 bytes
OFF_ACTIVE = 808
MODE_TRAJECTORY = 2

class RobotSHMTester:
    def __init__(self):
        try:
            # Mở file SHM trên Linux
            self.f = open('/dev/shm' + SHM_NAME, 'r+b')
            self.shm = mmap.mmap(self.f.fileno(), 0)
            print(f"✅ Đã kết nối tới SHM: {SHM_NAME}")
        except FileNotFoundError:
            print("❌ Lỗi: Chưa chạy ROS2 Control. SHM không tồn tại.")
            sys.exit(1)

    def sync_pose(self):
        """Đọc vị trí hiện tại và ghi vào Target để tránh robot bị giật khi bắt đầu"""
        self.shm.seek(96) # OFF_EE_POS
        current_pos = struct.unpack('3d', self.shm.read(24))
        self.shm.seek(120) # OFF_EE_RPY
        current_rpy = struct.unpack('3d', self.shm.read(24))
        
        # Ghi ngược vào Target
        self.shm.seek(OFF_TARGET_POS)
        self.shm.write(struct.pack('3d', *current_pos))
        self.shm.seek(OFF_TARGET_RPY)
        self.shm.write(struct.pack('3d', *current_rpy))
        print(f"🔄 Đã đồng bộ vị trí hiện tại: {current_pos}")

    def send_command(self, v_lin=[0.0, 0.0, 0.0], v_ang=[0.0, 0.0, 0.0], active=True):
        # 1. Ghi Mode Trajectory (Để Controller dùng solveIK_Velocity)
        self.shm.seek(OFF_MODE)
        self.shm.write(struct.pack('i', MODE_TRAJECTORY))
        
        # 2. Ghi Vận tốc Cartesian (6 doubles liên tiếp từ offset 200)
        self.shm.seek(OFF_TRAJ_VEL_LIN)
        self.shm.write(struct.pack('6d', *v_lin, *v_ang))
        
        # 3. Ghi cờ Active
        self.shm.seek(OFF_ACTIVE)
        self.shm.write(struct.pack('?', active))

    def stop(self):
        self.send_command([0,0,0], [0,0,0], active=False)
        self.shm.close()
        self.f.close()
        print("\n🛑 Đã dừng robot và đóng kết nối.")

# --- KỊCH BẢN TEST ---

def run_line_test(tester, speed=0.03, duration=5.0):
    """Đi thẳng theo trục X"""
    tester.sync_pose()
    print(f"🚀 Chạy đường thẳng X: {speed} m/s trong {duration}s")
    start = time.time()
    try:
        while time.time() - start < duration:
            tester.send_command(v_lin=[speed, 0.0, 0.0])
            time.sleep(0.01) # 100Hz
    except KeyboardInterrupt:
        pass
    tester.stop()

def run_circle_test(tester, radius=0.05, period=8.0):
    """Vẽ hình tròn mặt phẳng XY"""
    tester.sync_pose()
    print(f"🚀 Chạy hình tròn R={radius}m, Chu kỳ={period}s")
    omega = 2 * math.pi / period
    start = time.time()
    try:
        while time.time() - start < period:
            t = time.time() - start
            # vx = -R*w*sin(wt), vy = R*w*cos(wt)
            vx = -radius * omega * math.sin(omega * t)
            vy =  radius * omega * math.cos(omega * t)
            
            tester.send_command(v_lin=[vx, vy, 0.0])
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    tester.stop()

if __name__ == "__main__":
    test = RobotSHMTester()
    
    print("\n--- CHƯƠNG TRÌNH TEST ROBOT QUA SHM ---")
    print("1. Chạy đường thẳng trục X (Tiến)")
    print("2. Chạy đường thẳng trục Z (Lên)")
    print("3. Chạy hình tròn mặt phẳng XY")
    print("4. Thoát")
    
    cmd = input("Chọn chế độ (1-4): ")
    
    if cmd == '1':
        run_line_test(test, speed=0.04, duration=4.0)
    elif cmd == '2':
        run_line_test(test, speed=0.0, duration=4.0) # Sửa v_lin trong hàm nếu muốn test Z
        # Hoặc gọi nhanh: test.send_command([0,0,0.04]) ...
    elif cmd == '3':
        run_circle_test(test, radius=0.06, period=10.0)
    else:
        test.stop()