import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3Stamped, Vector3
from sensor_msgs.msg import Imu
import tf2_ros
import tf2_geometry_msgs # Required for tf2.toMsg(q)
from tf_transformations import quaternion_from_euler, euler_from_quaternion # Used for converting Euler to Quaternion and vice versa
import numpy as np
import time
import struct # For handling byte conversions with signed shorts

# --- 常數定義 (從您的 Header file 提取並調整) ---
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
SEND_DATA_CHECK = 1  # 对应 C++ 的 mode==1
READ_DATA_CHECK = 0  # 对应 C++ 的 mode==0

# IMU 相關常數
GYROSCOPE_RATIO = 0.00026644  # 弧度/LSB
ACCEL_RATIO = 1671.84        # LSB/(m/s^2)

SAMPLING_FREQ = 20.0 # 采样频率 (從 Quaternion_Solution.h 提取)

# --- 全局變數用於 Quaternion_Solution ---
# 這些是 C++ 原始程式碼中的 volatile float q0, q1, q2, q3 等
global_q0 = 1.0
global_q1 = 0.0
global_q2 = 0.0
global_q3 = 0.0
global_integralFBx = 0.0
global_integralFBy = 0.0
global_integralFBz = 0.0

# 2 * proportional gain (Kp) 和 2 * integral gain (Ki)
global_twoKp = 1.0
global_twoKi = 0.0

# --- 仿製的 C++ 函式 ---

def InvSqrt(number: float) -> float:
    """
    仿製 C++ 的 InvSqrt 函數 (快速平方根倒數)
    """
    # 這裡直接使用 Python 的 math.sqrt，效率更高且精確
    # 如果要完全仿製 C++ 的位元操作版本，會更複雜且可能不如內建函式精確
    # 但為了功能等價，這裡用 math.sqrt
    return 1.0 / np.sqrt(number) if number != 0 else 0.0

def Check_Sum(data_bytes: bytes, mode: int) -> int:
    """
    仿製 C++ 的 Check_Sum 函數 (BCC 異或校驗)
    data_bytes: 要校驗的位元組串
    mode: 0 for Receive data, 1 for Send data
    """
    check_sum = 0
    # C++ 程式碼中，Send_Data.tx 和 Receive_Data.rx 是整個陣列
    # Check_Sum 函數會接收 Count_Number (校驗的位元組數)
    
    # 根據原始 C++ 函式定義，Check_Sum(Count_Number, SEND_DATA_CHECK)
    # 會對 Send_Data.tx[0] 到 Send_Data.tx[Count_Number-1] 進行異或。
    # Check_Sum(22, READ_DATA_CHECK) 對 Receive_Data.rx[0] 到 Receive_Data.rx[21] 進行異或。

    # 在 Python 中，data_bytes 是傳入的「片段」
    # 因此直接遍歷傳入的 data_bytes 即可
    for byte_val in data_bytes:
        check_sum ^= byte_val
    return check_sum

def IMU_Trans(data_high: int, data_low: int) -> int:
    """
    仿製 C++ 的 IMU_Trans 函數
    將高位元組和低位元組組合成一個有符號的 16 位元整數。
    """
    transition_16 = (data_high << 8) | data_low
    # 確保是帶符號的 16 位元整數
    return struct.unpack('>h', struct.pack('>H', transition_16))[0]

def Odom_Trans(data_high: int, data_low: int) -> float:
    """
    仿製 C++ 的 Odom_Trans 函數
    將高位元組和低位元組組合成一個有符號的 16 位元整數，然後轉換為浮點數速度。
    """
    transition_16 = (data_high << 8) | data_low
    # 確保是帶符號的 16 位元整數
    signed_value = struct.unpack('>h', struct.pack('>H', transition_16))[0]
    
    # C++ 程式碼中的轉換邏輯: (transition_16 / 1000) + (transition_16 % 1000) * 0.001
    # 這實際上等同於直接除以 1000.0
    data_return = signed_value / 1000.0
    return data_return

def Quaternion_Solution(gx: float, gy: float, gz: float, ax: float, ay: float, az: float) -> Quaternion:
    """
    仿製 C++ 的 Quaternion_Solution 函數 (Madgwick 或 Mahony 濾波器核心)
    根據陀螺儀和加速度計數據更新四元數。
    這裡將直接更新全局四元數 q0, q1, q2, q3。
    """
    global global_q0, global_q1, global_q2, global_q3
    global global_integralFBx, global_integralFBy, global_integralFBz
    global global_twoKp, global_twoKi

    recipNorm = 0.0
    halfvx, halfvy, halfvz = 0.0, 0.0, 0.0
    halfex, halfey, halfez = 0.0, 0.0, 0.0
    qa, qb, qc = 0.0, 0.0, 0.0

    # 計算反饋，只有在加速度計測量有效時才進行 (避免 NaN)
    if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
        # 首先把加速度計采集到的值(三维向量)转化为单位向量，即向量除以模
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm

        # 把四元数换算成方向余弦中的第三行的三个元素
        # q0, q1, q2, q3 是當前的全局四元數
        halfvx = global_q1 * global_q3 - global_q0 * global_q2
        halfvy = global_q0 * global_q1 + global_q2 * global_q3
        halfvz = global_q0 * global_q0 - 0.5 + global_q3 * global_q3 # 原始 C++ 程式碼是 q0*q0 - 0.5f + q3*q3

        # 誤差是估計的重力方向和測量的重力方向的交叉乘積之和
        halfex = (ay * halfvz - az * halfvy)
        halfey = (az * halfvx - ax * halfvz)
        halfez = (ax * halfvy - ay * halfvx)

        # 計算並應用積分反饋（如果啟用）
        if global_twoKi > 0.0:
            global_integralFBx += global_twoKi * halfex * (1.0 / SAMPLING_FREQ)  # integral error scaled by Ki
            global_integralFBy += global_twoKi * halfey * (1.0 / SAMPLING_FREQ)
            global_integralFBz += global_twoKi * halfez * (1.0 / SAMPLING_FREQ)
            gx += global_integralFBx  # apply integral feedback
            gy += global_integralFBy
            gz += global_integralFBz
        else:
            global_integralFBx = 0.0  # prevent integral windup
            global_integralFBy = 0.0
            global_integralFBz = 0.0

        # Apply proportional feedback
        gx += global_twoKp * halfex
        gy += global_twoKp * halfey
        gz += global_twoKp * halfez

    # Integrate rate of change of quaternion
    gx *= (0.5 / SAMPLING_FREQ)  # pre-multiply common factors
    gy *= (0.5 / SAMPLING_FREQ)
    gz *= (0.5 / SAMPLING_FREQ)

    qa = global_q0
    qb = global_q1
    qc = global_q2

    global_q0 += (-qb * gx - qc * gy - global_q3 * gz)
    global_q1 += (qa * gx + qc * gz - global_q3 * gy)
    global_q2 += (qa * gy - qb * gz + global_q3 * gx)
    global_q3 += (qa * gz + qb * gy - qc * gx)

    # Normalise quaternion
    recipNorm = InvSqrt(global_q0 * global_q0 + global_q1 * global_q1 + global_q2 * global_q2 + global_q3 * global_q3)
    global_q0 *= recipNorm
    global_q1 *= recipNorm
    global_q2 *= recipNorm
    global_q3 *= recipNorm
    
    # 返回一個 geometry_msgs.msg.Quaternion 實例
    return Quaternion(x=global_q1, y=global_q2, z=global_q3, w=global_q0) # 注意 ROS 四元數的順序是 x,y,z,w

# --- ROS 2 節點類別 ---
class TurnOnRobot(Node):
    def __init__(self):
        super().__init__('turn_on_robot')
        self.get_logger().info("ROS 2 node 'turn_on_robot' started.")

        # --- 節點參數 (Odom 和 TF) ---
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.robot_frame_id = self.declare_parameter('robot_frame_id', 'base_footprint').value

        # --- Publisher ---
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.voltage_publisher = self.create_publisher(Vector3Stamped, 'power_voltage', 10) # 使用 Vector3Stamped 模擬電壓

        # --- Subscriber ---
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --- TF Broadcaster ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- 機器人狀態變數 ---
        self.robot_pos_x = 0.0
        self.robot_pos_y = 0.0
        # self.robot_yaw 不需要獨立維護，由 Quaternion_Solution 的 global_q 決定

        self.robot_vel_x = 0.0
        self.robot_vel_y = 0.0
        self.robot_vel_z = 0.0 # 角速度

        self.power_voltage = 12.0 # 模擬電池電壓

        # IMU 數據 (模擬原始值和轉換後的值)
        self.mpu6050_accel_x_raw = 0
        self.mpu6050_accel_y_raw = 0
        self.mpu6050_accel_z_raw = 0
        self.mpu6050_gyros_x_raw = 0
        self.mpu6050_gyros_y_raw = 0
        self.mpu6050_gyros_z_raw = 0

        self.imu_linear_acceleration = Vector3()
        self.imu_angular_velocity = Vector3()
        self.imu_orientation = Quaternion() # 機器人姿態，由 Quaternion_Solution 更新

        # --- Odom 協方差矩陣 (與 C++ 程式碼保持一致) ---
        self.odom_pose_covariance = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3
        ]
        self.odom_pose_covariance2 = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0, # 注意這裡原始 C++ 的 y 協方差是 1e-3，不是 1e-9
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9
        ]
        self.odom_twist_covariance = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3
        ]
        self.odom_twist_covariance2 = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0, # 注意這裡原始 C++ 的 y 協方差是 1e-3，不是 1e-9
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9
        ]


        # --- 定時器來模擬數據更新和發布 ---
        self.timer_period = 1.0 / SAMPLING_FREQ # 根據 SAMPLING_FREQ (20.0f) 設定
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def get_sensor_data_new(self) -> bool:
        """
        模擬從下位機接收數據。
        在實際應用中，這裡會是串口讀取邏輯。
        我們將隨機生成一些模擬數據來測試 Odometry 和 IMU 發布。
        """
        # 為了演示，我們隨機生成一些數值，並將它們轉換回“原始”位元組形式
        # 讓它們在 Odom_Trans 和 IMU_Trans 中能夠被處理

        # 模擬速度 (例如 -0.5 到 0.5 m/s, -0.5 到 0.5 rad/s)
        # 這裡的模擬數據是為了讓程式碼能夠運行，真實情況會從串口讀取
        # 為了讓 Odometry 能夠積累，這裡根據上次的速度來模擬當前速度
        # 否則每次都是隨機速度會讓機器人位置亂跳
        self.robot_vel_x = np.random.uniform(-0.1, 0.1) # 模擬隨機變化
        self.robot_vel_y = np.random.uniform(-0.05, 0.05)
        self.robot_vel_z = np.random.uniform(-0.02, 0.02) # 角速度

        # 模擬 IMU 原始值
        # 這裡為了簡化，直接給定一些隨機的原始值，
        # 這些值經過 IMU_Trans 和除以 ACCEL_RATIO/GYROSCOPE_RATIO 會變為物理單位
        self.mpu6050_accel_x_raw = int(np.random.uniform(-1000, 1000))
        self.mpu6050_accel_y_raw = int(np.random.uniform(-1000, 1000))
        self.mpu6050_accel_z_raw = int(np.random.uniform(16000, 17000)) # 假設 Z 軸接近重力
        self.mpu6050_gyros_x_raw = int(np.random.uniform(-50, 50))
        self.mpu6050_gyros_y_raw = int(np.random.uniform(-50, 50))
        self.mpu6050_gyros_z_raw = int(self.robot_vel_z / GYROSCOPE_RATIO) # 讓 IMU 的 Z 角速度與 Odom 的 Z 角速度一致

        # 模擬電池電壓
        self.power_voltage = np.random.uniform(11.5, 12.5)

        # 模擬構建接收數據的位元組陣列，以便測試 Check_Sum 等函數
        receive_data_bytes = bytearray(24)
        receive_data_bytes[0] = FRAME_HEADER
        receive_data_bytes[1] = 0 # Flag_Stop 預留位 (uint8_t)

        # 速度數據 (轉換為 int16_t 再拆分)
        # C++ 程式碼中是 float * 1000 轉為 short
        vel_x_int = int(self.robot_vel_x * 1000)
        vel_y_int = int(self.robot_vel_y * 1000)
        vel_z_int = int(self.robot_vel_z * 1000)

        # 使用 struct 處理有符號數的位元組轉換
        # short 是 2 bytes
        struct.pack_into('>h', receive_data_bytes, 2, vel_x_int) # offset 2 for X
        struct.pack_into('>h', receive_data_bytes, 4, vel_y_int) # offset 4 for Y
        struct.pack_into('>h', receive_data_bytes, 6, vel_z_int) # offset 6 for Z

        # IMU 原始數據
        struct.pack_into('>h', receive_data_bytes, 8, self.mpu6050_accel_x_raw) # offset 8 for Accel X
        struct.pack_into('>h', receive_data_bytes, 10, self.mpu6050_accel_y_raw) # offset 10 for Accel Y
        struct.pack_into('>h', receive_data_bytes, 12, self.mpu6050_accel_z_raw) # offset 12 for Accel Z
        struct.pack_into('>h', receive_data_bytes, 14, self.mpu6050_gyros_x_raw) # offset 14 for Gyro X
        struct.pack_into('>h', receive_data_bytes, 16, self.mpu6050_gyros_y_raw) # offset 16 for Gyro Y
        struct.pack_into('>h', receive_data_bytes, 18, self.mpu6050_gyros_z_raw) # offset 18 for Gyro Z

        # 電壓數據 (C++ 程式碼中電壓也是 short * 1000，然後 /1000 + %1000*0.001)
        voltage_int = int(self.power_voltage * 1000)
        struct.pack_into('>h', receive_data_bytes, 20, voltage_int) # offset 20 for Voltage

        # 計算校驗和 (C++ 是對前 22 個位元組進行異或校驗)
        computed_checksum = Check_Sum(receive_data_bytes[0:22], READ_DATA_CHECK)
        receive_data_bytes[22] = computed_checksum
        receive_data_bytes[23] = FRAME_TAIL

        # 模擬校驗和通過 (在實際串口通訊中需要檢查)
        # 這裡直接檢查幀頭、幀尾和計算出的校驗和是否一致
        if receive_data_bytes[0] == FRAME_HEADER and \
           receive_data_bytes[23] == FRAME_TAIL and \
           receive_data_bytes[22] == computed_checksum: # 由於我們是模擬生成數據，這裡會總是通過

            # 解析數據 (與 C++ 程式碼對應)
            self.robot_vel_x = Odom_Trans(receive_data_bytes[2], receive_data_bytes[3])
            self.robot_vel_y = Odom_Trans(receive_data_bytes[4], receive_data_bytes[5])
            self.robot_vel_z = Odom_Trans(receive_data_bytes[6], receive_data_bytes[7])

            # IMU 原始數據解析
            self.mpu6050_accel_x_raw = IMU_Trans(receive_data_bytes[8], receive_data_bytes[9])
            self.mpu6050_accel_y_raw = IMU_Trans(receive_data_bytes[10], receive_data_bytes[11])
            self.mpu6050_accel_z_raw = IMU_Trans(receive_data_bytes[12], receive_data_bytes[13])
            self.mpu6050_gyros_x_raw = IMU_Trans(receive_data_bytes[14], receive_data_bytes[15])
            self.mpu6050_gyros_y_raw = IMU_Trans(receive_data_bytes[16], receive_data_bytes[17])
            self.mpu6050_gyros_z_raw = IMU_Trans(receive_data_bytes[18], receive_data_bytes[19])

            # 轉換為國際單位
            self.imu_linear_acceleration.x = self.mpu6050_accel_x_raw / ACCEL_RATIO
            self.imu_linear_acceleration.y = self.mpu6050_accel_y_raw / ACCEL_RATIO
            self.imu_linear_acceleration.z = self.mpu6050_accel_z_raw / ACCEL_RATIO

            self.imu_angular_velocity.x = self.mpu6050_gyros_x_raw * GYROSCOPE_RATIO
            self.imu_angular_velocity.y = self.mpu6050_gyros_y_raw * GYROSCOPE_RATIO
            self.imu_angular_velocity.z = self.mpu6050_gyros_z_raw * GYROSCOPE_RATIO

            self.power_voltage = Odom_Trans(receive_data_bytes[20], receive_data_bytes[21]) 

            return True
        return False

    def publish_odom(self):
        """
        發布 Odometry 數據。
        """
        global global_q0, global_q1, global_q2, global_q3
        current_time = self.get_clock().now()
        
        # 姿態四元數直接來自 Quaternion_Solution 更新的全局變數
        odom_quat = Quaternion(x=global_q1, y=global_q2, z=global_q3, w=global_q0)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.robot_pos_x
        odom.pose.pose.position.y = self.robot_pos_y
        odom.pose.pose.position.z = 0.0 # 假設在平面上移動
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = self.robot_frame_id
        odom.twist.twist.linear.x = self.robot_vel_x
        odom.twist.twist.linear.y = self.robot_vel_y
        odom.twist.twist.linear.z = 0.0 # 假設無垂直速度
        odom.twist.twist.angular.x = self.imu_angular_velocity.x
        odom.twist.twist.angular.y = self.imu_angular_velocity.y
        odom.twist.twist.angular.z = self.robot_vel_z # 使用 Odom 的角速度

        # 協方差矩陣選擇
        if abs(self.robot_vel_x) < 1e-6 and abs(self.robot_vel_y) < 1e-6 and abs(self.robot_vel_z) < 1e-6:
            odom.pose.covariance = self.odom_pose_covariance2
            odom.twist.covariance = self.odom_twist_covariance2
        else:
            odom.pose.covariance = self.odom_pose_covariance
            odom.twist.covariance = self.odom_twist_covariance
            
        self.odom_publisher.publish(odom)

        # 同時發布 TF (odom -> base_footprint)
        t = tf2_ros.TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.robot_frame_id
        t.transform.translation.x = self.robot_pos_x
        t.transform.translation.y = self.robot_pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(t)

    def publish_imu_sensor(self):
        """
        發布 IMU 數據。
        """
        global global_q0, global_q1, global_q2, global_q3
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link" # 或者其他 IMU 的 frame_id

        imu_msg.linear_acceleration = self.imu_linear_acceleration
        imu_msg.angular_velocity = self.imu_angular_velocity
        # 姿態四元數，直接使用 Quaternion_Solution 更新的全局變數
        imu_msg.orientation.x = global_q1
        imu_msg.orientation.y = global_q2
        imu_msg.orientation.z = global_q3
        imu_msg.orientation.w = global_q0

        # 協方差矩陣 (簡化，實際應用中應有真實值)
        imu_msg.orientation_covariance = [
            1e-6, 0, 0,
            0, 1e-6, 0,
            0, 0, 1e-6
        ]
        imu_msg.angular_velocity_covariance = [
            1e-6, 0, 0,
            0, 1e-6, 0,
            0, 0, 1e-6
        ]
        imu_msg.linear_acceleration_covariance = [
            1e-6, 0, 0,
            0, 1e-6, 0,
            0, 0, 1e-6
        ]

        self.imu_publisher.publish(imu_msg)

    def publish_voltage(self):
        """
        發布電池電壓。
        """
        voltage_msg = Vector3Stamped()
        voltage_msg.header.stamp = self.get_clock().now().to_msg()
        voltage_msg.header.frame_id = "battery_link" # 或其他電壓的 frame_id
        voltage_msg.vector.x = self.power_voltage
        voltage_msg.vector.y = 0.0
        voltage_msg.vector.z = 0.0
        self.voltage_publisher.publish(voltage_msg)

    def cmd_vel_callback(self, twist_aux: Twist):
        """
        接收 cmd_vel 指令並模擬發送給下位機。
        """
        # 模擬將速度轉換為串口發送的位元組陣列
        send_data_bytes = bytearray(11)
        send_data_bytes[0] = FRAME_HEADER
        send_data_bytes[1] = 0 # 預留位 (AutoRecharge 變數在程式 B 中是 0)
        send_data_bytes[2] = 0 # 預留位

        # 線速度 (X, Y) 和角速度 (Z)，放大 1000 倍並轉換為 int16_t
        linear_x_int = int(twist_aux.linear.x * 1000)
        linear_y_int = int(twist_aux.linear.y * 1000)
        angular_z_int = int(twist_aux.angular.z * 1000)

        # 使用 struct 處理有符號數的位元組轉換
        struct.pack_into('>h', send_data_bytes, 3, linear_x_int) # offset 3 for X
        struct.pack_into('>h', send_data_bytes, 5, linear_y_int) # offset 5 for Y
        struct.pack_into('>h', send_data_bytes, 7, angular_z_int) # offset 7 for Z

        # 計算校驗和 (針對前 9 個位元組: Send_Data.tx[0] 到 Send_Data.tx[8])
        computed_checksum = Check_Sum(send_data_bytes[0:9], SEND_DATA_CHECK)
        send_data_bytes[9] = computed_checksum
        send_data_bytes[10] = FRAME_TAIL

        # 實際應用中，這裡會透過串口發送 `send_data_bytes`
        self.get_logger().info(f"Received cmd_vel: linear_x={twist_aux.linear.x:.2f}, angular_z={twist_aux.angular.z:.2f}")
        self.get_logger().info(f"Simulating sending to lower machine: {send_data_bytes.hex()}")

    def control_loop(self):
        """
        主控制迴圈，模擬數據接收、里程計更新和話題發布。
        """
        current_time = self.get_clock().now()
        sampling_time = (current_time - self.last_time).nanoseconds / 1e9 # 轉換為秒

        if self.get_sensor_data_new(): # 模擬成功接收到下位機數據
            # 計算位移 (積分速度)
            # 從全局四元數獲取當前 yaw 角度，用於位移計算
            # global_q0, global_q1, global_q2, global_q3 是 W,X,Y,Z 順序 (tf_transformations 是 x,y,z,w)
            _, _, current_yaw = euler_from_quaternion([global_q1, global_q2, global_q3, global_q0])
            
            self.robot_pos_x += (self.robot_vel_x * np.cos(current_yaw) - self.robot_vel_y * np.sin(current_yaw)) * sampling_time
            self.robot_pos_y += (self.robot_vel_x * np.sin(current_yaw) + self.robot_vel_y * np.cos(current_yaw)) * sampling_time
            
            # 更新全局四元數 (由 Quaternion_Solution 處理)
            # 將 IMU 讀取的數據傳遞給姿態解算函數
            Quaternion_Solution(
                self.imu_angular_velocity.x, self.imu_angular_velocity.y, self.imu_angular_velocity.z,
                self.imu_linear_acceleration.x, self.imu_linear_acceleration.y, self.imu_linear_acceleration.z
            )
            
            self.publish_odom()
            self.publish_imu_sensor()
            self.publish_voltage()

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    turn_on_robot_node = TurnOnRobot()
    try:
        rclpy.spin(turn_on_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        turn_on_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()