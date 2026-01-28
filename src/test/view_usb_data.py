import serial
import struct
import time
import threading

# Cấu hình cổng ảo
SERIAL_PORT = '/tmp/ttyV1'
BAUDRATE = 115200

# Định dạng Struct
CMD_BODY_FMT = "<fffffffB" 
FB_FMT = "<HfffffffffffffB"

# Biến toàn cục
current_joints = [0.0] * 6
current_gripper = 0.0
lock = threading.Lock()

# BIẾN THỜI GIAN ĐỂ GIẢI QUYẾT PHỤ THUỘC TẦN SỐ
last_packet_time = None

def calculate_checksum(data_bytes):
    return sum(data_bytes) & 0xFF

def receive_thread(ser):
    global current_joints, current_gripper, last_packet_time
    print(f"[*] Simulator: Frequency Independent Mode Active")
    
    while True:
        try:
            # 1. Săn Header AA 55
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA: continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55: continue
            
            # 2. Đọc dữ liệu
            remaining_data = ser.read(29)
            if len(remaining_data) < 29: continue
                
            unpacked = struct.unpack(CMD_BODY_FMT, remaining_data)
            vels = unpacked[0:6]
            grip = unpacked[6]
            checksum_recv = unpacked[7]

            if calculate_checksum(b'\xAA\x55' + remaining_data[:-1]) == checksum_recv:
                with lock:
                    # --- CHIẾN THUẬT KHÔNG PHỤ THUỘC TẦN SỐ ---
                    current_time = time.perf_counter() # Dùng đồng hồ chính xác cao
                    
                    if last_packet_time is None:
                        # Lần đầu tiên nhận gói, chỉ ghi nhận thời gian
                        dt = 0.0
                    else:
                        # Tính thời gian thực tế trôi qua kể từ gói trước (giây)
                        dt = current_time - last_packet_time
                    
                    last_packet_time = current_time

                    # Giới hạn dt để tránh robot nhảy vọt nếu bị nghẽn mạng Serial
                    if dt > 0.1: dt = 0.01 

                    # Tích phân: Quãng đường = Vận tốc * Thời gian thực tế
                    for i in range(6):
                        # Chỉ di chuyển nếu vận tốc đủ lớn (Deadzone)
                        v = vels[i] if abs(vels[i]) > 0.0001 else 0.0
                        
                        # Cập nhật vị trí
                        current_joints[i] += v * dt
                        
                        # Giới hạn khớp
                        current_joints[i] = max(min(current_joints[i], 3.14), -3.14)
                    
                    current_gripper = grip
                
                # In feedback
                pos_str = ", ".join([f"{j:6.3f}" for j in current_joints])
                print(f"\r[FREQ-INDEP] DT: {dt:.4f}s | Joints: [{pos_str}]", end="", flush=True)

        except Exception as e:
            time.sleep(0.01)

def send_thread(ser):
    """Gửi feedback 30Hz về ROS 2"""
    while True:
        try:
            header = 0x55AA 
            with lock:
                fb_list = [header] + current_joints + [0.0]*6 + [current_gripper]
            packed_data = struct.pack("<Hfffffffffffff", *fb_list)
            ser.write(packed_data + struct.pack("<B", calculate_checksum(packed_data)))
            time.sleep(0.033) 
        except: pass

def main():
    try:
        # Tăng tốc độ đọc của Python để bắt kịp 1000Hz từ C++
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.001)
        ser.reset_input_buffer()
        threading.Thread(target=receive_thread, args=(ser,), daemon=True).start()
        threading.Thread(target=send_thread, args=(ser,), daemon=True).start()
        while True: time.sleep(1)
    except KeyboardInterrupt:
        ser.close()

if __name__ == "__main__":
    main()