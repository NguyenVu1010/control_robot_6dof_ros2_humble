#!/bin/bash

# 1. Xác định đường dẫn tương đối
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WS_ROOT=$(cd "$SCRIPT_DIR/../../" && pwd)

echo "========================================================="
echo "   HỆ THỐNG GIẢ LẬP SERIAL - FIX PERMISSION"
echo "========================================================="

# 2. Dọn dẹp tuyệt đối (Dùng sudo để xóa các link cứng đầu)
echo "[1/5] Cleaning up old processes and locks..."
sudo killall -9 socat python3 ros2_control_node 2>/dev/null
# Xóa các file rác và vùng nhớ chia sẻ
sudo rm -f /dev/ttyUSB0 /tmp/ttyV0 /tmp/ttyV1
sudo rm -f /dev/shm/robot_control_shm

# 3. Khởi tạo cổng ảo (Dùng sudo để socat có quyền tạo link đồng nhất)
echo "[2/5] Creating Virtual Serial Ports (/tmp/ttyV0 <-> /tmp/ttyV1)..."
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1 &
sleep 2 # Chờ socat tạo file

# 4. Thiết lập liên kết /dev/ttyUSB0 cho ROS 2
echo "[3/5] Setting up /dev/ttyUSB0 and permissions..."
if [ -e /tmp/ttyV0 ]; then
    sudo ln -s /tmp/ttyV0 /dev/ttyUSB0
    # Cấp quyền đọc ghi cho tất cả mọi người (666)
    sudo chmod 666 /dev/ttyUSB0
    sudo chmod 666 /tmp/ttyV0
    sudo chmod 666 /tmp/ttyV1
    echo "Done: /dev/ttyUSB0 is ready."
else
    echo "ERROR: socat failed to create /tmp/ttyV0"
    exit 1
fi

# 5. Khởi chạy Monitor (Cửa sổ mới)
echo "[4/5] Launching Serial Monitor..."
gnome-terminal --title="SERIAL MONITOR" -- bash -c "python3 $SCRIPT_DIR/view_usb_data.py; exec bash"

# 6. Khởi chạy ROS 2
echo "[5/5] Launching ROS 2 System..."
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
else
    echo "ERROR: install/setup.bash not found! Please build your workspace."
    exit 1
fi

# Chạy ROS 2 Launch
ros2 launch my_arm_robot robot_bringup.launch.py \
    use_mock_hardware:=false \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200