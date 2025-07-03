from collections import deque
from math import floor
from select import select
import socket
from threading import Thread
import serial
import time
import sys
import struct

class BaseBridge():
    # Initialize this class
    def __init__(self):
        # minimum threshold (rad/s) for motor rotates
        self.threshold_wheel = 0.45
        # 除錯資料列印
        self.debug_mode = False
        # 接收 buffer
        buf = b''
        # 設定串列通訊
        try:
            # Communication parameter
            self.device_port = '/dev/minibot_base'
            self.baudrate = 115200
            # 通訊逾時調整成10ms，以免等待資料卡住整個程式
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=0.01)
            time.sleep(1)
        except serial.serialutil.SerialException:
            print(f"[ERROR] [INIT] Can not receive data from the port: {self.device_port}. Did you specify the correct port?")
            sys.exit(0)
        # 設定UDP通訊
        udp_ip = '0.0.0.0'
        udp_port = 12584
        self.s_udp_ip = '127.0.0.1'
        self.s_udp_port = 12583
        # Base's specs
        self.wheelSep = 0.158
        self.wheelRad = 0.032
        # Create socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.sock.setblocking(0)
        # 通訊建立列印訊息
        print("[INFO] [INIT] Communication success !")
        # Timers
        self.current_time_base = time.time()
        self.previous_time_base_cmd = self.current_time_base
        self.previous_time_base_pub = self.current_time_base
        self.current_time_nocs = self.current_time_base
        self.previous_time_nocs = self.current_time_base
        # 寫入初始皆為0的訊號
        self.wrtie_spd(0.0, 0.0)
        # Deque to switch between two thread
        # Put the signal receive from base
        self.base_signal_deque = deque(maxlen=3)
        # Put the signal that receive from UDP (NOCS)
        self.nocs_signal_deque = deque(maxlen=3)
        # 等待底盤反應
        time.sleep(1)
    # 輔助函式：計算 XOR 總和檢查碼，與C++程式相同
    def calculate_checksum(self, payload_bytes: list):
        # 計算給定位元組串的 XOR 總和檢查碼。
        checksum = 0
        for byte in payload_bytes:
            checksum ^= byte
        return checksum
    # Function to write speed
    def wrtie_spd(self, vx: float, vrz: float):
        # 反解運動學
        WR = (vx + self.wheelSep / 2.0 * vrz) / self.wheelRad
        WL = (vx - self.wheelSep / 2.0 * vrz) / self.wheelRad
        # 解算出太小的數值，以最小容許的數值旋轉
        def clamp_to_threshold(value, threshold=self.threshold_wheel):
            if abs(value) < threshold:
                if value > 0:
                    return threshold
                elif value < 0:
                    return -threshold
            return value
        if (abs(WR) < self.threshold_wheel):
            WR = clamp_to_threshold(WR)
        if (abs(WL) < self.threshold_wheel):
            WL = clamp_to_threshold(WL)
        # 將資料轉換為32bit浮點數
        WR_send_ba = struct.pack("f", WR)
        WL_send_ba = struct.pack("f", WL)
        # 除錯用
        if self.debug_mode:
            print(f"[DEBUG] [BASE] Send vx = {str(vx)}, vrz = {str(vrz)}, WR = {str(WR)}, WL = {str(WL)}")
        # 將資料與Header、Function Code、Footer合併
        output = b'SV' + WR_send_ba + WL_send_ba + b'E'
        # 除錯用
        if self.debug_mode:
            print(f"[DEBUG] [BASE] Send packet = {output}")
        # 寫入初始命令
        self.serial.write(output)
    # 底盤串列通訊執行緒
    def base_control_thread(self):
        # 切換狀態的變數
        step = 0
        buf = b""
        cmd_vx = 0.0
        cmd_vrz = 0.0
        cmd_vy = 0.0
        safestop = False
        print("[INFO] [BASE] Base receiver thread started!")
        while(True):
            self.current_time_base = time.time()
            # 發送部分
            # Have data in the deque
            if(len(self.nocs_signal_deque) > 0):
                # 抽出Buffer最早的資料
                cmd_vx, cmd_vy, cmd_vrz = self.nocs_signal_deque.popleft()
                # Reset timer
                self.previous_time_base_cmd = self.current_time_base
                safestop = False
            else:
                # 超過500ms上游沒有控制命令，則安全停車
                if(self.current_time_base - self.previous_time_base_cmd > 0.5 and not safestop):
                    cmd_vx = 0.0
                    cmd_vy = 0.0
                    cmd_vrz = 0.0
                    self.wrtie_spd(cmd_vx, cmd_vrz)
                    print("[INFO] [BASE] Safe stop!")
                    safestop = True
                # 沒有命令時，每50ms發送目前速度訊號以確保取得底盤狀態回傳
                if(self.current_time_base - self.previous_time_base_pub > 0.05):
                    # 再送出一次當前速度
                    self.wrtie_spd(cmd_vx, cmd_vrz)
                    # Reset timer
                    self.previous_time_base_pub = self.current_time_base
            # 接收部分
            # Step 0: Wait data coming
            if(step == 0):
                # Try to read buffer
                buf = self.serial.read(1)
                # Check vaild header
                if buf == b'S':
                    if(self.debug_mode):
                        print("[DEBUG] [BASE] Header vaild!")
                    step = 1
            # Step 1: Second header check and packet classification
            elif(step == 1):
                buf = self.serial.read(1)
                # Battery message
                if buf == b'B':
                    step = 2
                # Velocity message
                elif buf == b'V':
                    step = 3
            # Step 2: Battery message vaildation
            elif(step == 2):
                buf = self.serial.read(5)
                # Length check
                if(buf < 5):
                    # Not match, drop it
                    step = 0
                else:
                    # Footer check
                    if (buf[-1] == ord('E')):
                        print(f"[INFO] [BASE] Got battery message and vaild!")
                        step = 0
                    else:
                        print(f"[WARN] [BASE] Packet battery footer vaildation fail!, footer = {buf[-1]}")
                        step = 0
            # Step 3: Velocity message vaildation
            elif(step == 3):
                buf = self.serial.read(13)
                # Length check
                if(len(buf) < 13):
                    # Not match, drop it
                    step = 0
                else:
                    # Footer check
                    if (buf[-1] == ord('E')):
                        if self.debug_mode:
                            print(f"[DEBUG] [BASE] Got velocity message and vaild!")
                        step = 4
                    else:
                        print(f"[WARN] [BASE] Packet velocity footer vaildation fail!, footer = {buf[-1]}")
                        step = 0
            # Step 4: Velocity handler
            elif(step == 4):
                try:
                    # 解碼資料回浮點數
                    VR = struct.unpack('f', buf[0:4])[0]
                    VL = struct.unpack('f', buf[4:8])[0]
                    gyro_z = struct.unpack('f', buf[8:12])[0]
                    # VR、VL、gyro_z，小於 0.01 時，使其值為 0
                    # VR, VL, gyro_z = map(lambda v: 0.0 if abs(v) < 0.01 else v, [VR, VL, gyro_z])
                    # 解算差動車體運動學 (Unit of VR and VS are rps)
                    VR *= self.wheelRad
                    VL *= self.wheelRad
                    Vyaw = (VR - VL) / self.wheelSep
                    Vx = (VR + VL) / 2.0
                    if self.debug_mode:
                        print(f"[DEBUG] [BASE] VR = {VR}, VL = {VL}, gyro_z = {gyro_z}")
                    # 放進Deque
                    self.base_signal_deque.append((Vx, 0.0, Vyaw))
                    step = 0
                except Exception as e:
                    print(f"[ERROR] [BASE] Velocity parse error: {e}")
                    step = 0
    # NOCS 溝通用
    def nocs_comm_thread(self):
        # 切換狀態的變數
        step = 0
        # Buffer
        buf = b""
        online = False
        print("[INFO] [BRIDGE] Bridge thread started!")
        # Generate the timestamp of 32bit
        def gotTimestamp():
            # Time stamp by ms
            timestamp = floor(time.time() * 1000)
            # Keep only the last 32 bits of the timestamp using a bitwise AND
            timestamp_32bit = timestamp & 0xFFFFFFFF
            return timestamp_32bit
        while(True):
            self.current_time_nocs = time.time()
            # 發送部分
            # deque 有需要發送的資料，並且NOCS為上線狀態
            if(len(self.base_signal_deque) > 0 and online):
                Vx, Vy, Vrz = self.base_signal_deque.popleft()
                # Time stamp by ms
                timestamp = gotTimestamp()
                # Pack the timestamp packet
                # 小端序無符號整數( <I )
                b_timestamp = struct.pack("<I", timestamp)
                payload = b_timestamp + \
                          struct.pack("<f", Vx) + \
                          struct.pack("<f", Vy) + \
                          b'\x00' * 12 + \
                          struct.pack("<f", Vrz)
                chksum = self.calculate_checksum(payload)
                # 28 bytes = 0x1C
                packet = b'Ar\x1C' + payload + chksum.to_bytes(1, 'little') + b'pk'
                self.sock.sendto(packet, (self.s_udp_ip, self.s_udp_port))
                if(self.debug_mode):
                    print("[DEBUG] [BRIDGE] Robot speed send!")
            # 計時部分
            # 5 秒沒有新指令接收
            if(self.current_time_nocs - self.previous_time_nocs > 5.0 and online):
                online = False
                print("[WARN] [BRIDGE] Wait ping timeout!")
            # 接收部分
            # Step 0: Wait request
            if(step == 0):
                # Try to read packet
                ready = select([self.sock], [], [], 0.01)
                if ready[0]:
                    buf, addr = self.sock.recvfrom(1024) 
                    # Received data at least need to include header + length + checksum + footer
                    if(self.debug_mode):
                        print(f"[DEBUG] [BRIDGE] packet = {buf}, from = {addr}")
                    if len(buf) > 6:
                        step = 1
            # Step 1: Common Header and length check
            elif(step == 1):
                # Is common header
                header = buf[0:2]
                if header == b"As" or header == b"Ac":
                    # Length check
                    r_len = buf[2]
                    # Header(2) + Length(1) + Payload(N) + Checksum(1) + Footer(2)
                    if(len(buf) != r_len + 6):
                        print(f"[WARN] [BRIDGE] Packet length vaildation fail! length = {r_len}")
                        step = 0
                    else:
                        step = 2
                else:
                    print(f"[WARN] [BRIDGE] Packet header vaildation fail! header = {buf[0:2]}")
                    step = 0
            # Step 2: Chksum, footer vaildation and parse data
            elif(step == 2):
                payload = buf[3:-3]
                footer = buf[-2:]
                chksum = buf[-3]
                # Footer vaildation
                if(footer == b"pk"):
                    # Checksum vaildation
                    v_chksum = self.calculate_checksum(payload)
                    if(v_chksum != chksum):
                        print(f"[WARN] [BRIDGE] Packet chesksum vaildation fail!, chksum = {chksum}, vaild = {v_chksum}")
                        step = 0
                    else:
                        header = buf[0:2]
                        # Request packet
                        if(header == b"As"):
                            step = 10
                        # Control speed packet
                        elif(header == b"Ac"):
                            step = 11
                else:
                    print(f"[WARN] [BRIDGE] Packet footer vaildation fail!, footer = {footer}")
                    step = 0
            # Step 10: Handle request packet
            elif(step == 10):
                # Ping request
                if payload == b'Ping':
                    # 將Nocs翻轉為上線狀態
                    online = True
                    # Reset timer
                    self.previous_time_nocs = self.current_time_nocs
                    print("[INFO] [BRIDGE] Got ping! Now status set to online")
                # Time request
                elif payload == b'Time':
                    # 回應時間請求，發送當前時間戳
                    timestamp = gotTimestamp()
                    # Pack the timestamp packet
                    # 小端序無符號整數( <I )
                    b_timestamp = struct.pack("<I", timestamp)
                    chksum = self.calculate_checksum(b_timestamp)
                    # Generate packet
                    # At + len(4) + timestamp + chksum + pk
                    packet = b'At\x04' + b_timestamp + chksum.to_bytes(1, 'little') + b'pk'
                    # Send packet
                    self.sock.sendto(packet, (self.s_udp_ip, self.s_udp_port))
                    print("[INFO] [BRIDGE] Time request send!")
                # Finish, back to receive next packet
                step = 0
            # Step 11: Handle control speed packet
            elif(step == 11):
                payload = buf[3:-3]
                vx = struct.unpack("<f", payload[0:4])[0]
                vy = struct.unpack("<f", payload[4:8])[0]
                vrz = struct.unpack("<f", payload[20:24])[0]
                self.nocs_signal_deque.append((vx, vy, vrz))
                # Finish, back to receive next packet
                step = 0

    # The entry point can call from others process
    def start_bridge(self):
        # Initialize the class and threads
        base_thread = Thread(target=self.base_control_thread, daemon=True)
        nocs_thread = Thread(target=self.nocs_comm_thread, daemon=True)
        base_thread.start()
        nocs_thread.start()
        # Join the second thread, don't exit the program
        nocs_thread.join()

if __name__ == "__main__":
    base_bridge = BaseBridge()
    base_bridge.start_bridge()

