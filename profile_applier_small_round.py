import os
import subprocess
import xml.etree.ElementTree as ET
import yaml

class Profile_params():
    # NOCS position (Use in future)
    nocs_pos_x = 0.0
    nocs_pos_y = 0.0
    nocs_pos_z = 0.0
    nocs_pos_r = 0.0
    nocs_pos_p = 0.0
    nocs_pos_y = 0.0
    # Robot size (Robot body's size) (注意：很多參數會依據機器人身體中心為原點計算)
    robot_size_x = 0.205
    robot_size_y = 0.190
    robot_size_z = 0.110
    # Laser position (Robot body's center --> Laser)
    laser_pos_x = -0.065
    laser_pos_y = 0.0
    laser_pos_z = 0.105
    laser_pos_r = 0.0
    laser_pos_p = 0.0
    laser_pos_yw = 0.0
    # IMU position (Robot body's center --> IMU)
    imu_pos_x = 0.065
    imu_pos_y = -0.055
    imu_pos_z = 0.055
    imu_pos_r = 0.0
    imu_pos_p = 0.0
    imu_pos_yw = 0.0
    # Wheel description (Robot body's center --> wheels)
    wheel_pos_x = 0.04  # X distance between wheel shaft and robot center
    wheel_pos_z = -0.073  # Z distance between wheel shaft and robot center
    wheel_separate = 0.079  # Distance between two wheels
    wheel_diameter = 0.064  # Size of the wheels
    wheel_thick = 0.025  # Thick of single wheel
    # Navigation parameters
    inflation_radius = 0.20  # The nearest distance of the obstacle
    map_filename = "250701_S01F1.yaml"    # Map yaml config filename under navigation folder
    max_linear_vel = 0.3      # Max linear velocity (Orig: 0.4)
    max_angular_vel = 1.0     # Max angular velocity (Orig: 0.75)
    min_linear_vel = 0.0     # Min linear velocity (Orig: 0.0), keep it 0.0 if you use 2wd or you cannot do the self-rotation!
    min_angular_vel = 0.3    # Min angular velocity (Orig: 0.0)
    xy_goal_tolerance = 0.25   # Robot position tolerance (Orig: 0.25)
    yaw_goal_tolerance = 0.25  # Robot facing tolerance (Orig: 0.25)
    # Auto calculated (Don't touch)
    robot_radius = 0.0
    wheel_radius = 0.0
    # 在這裡會計算自動更新的參數的數值
    def __init__(self):
        self.robot_radius = (self.robot_size_x / 2.0) + self.wheel_pos_x
        self.wheel_radius = self.wheel_diameter / 2
        # 當導航PC與感應器模組化後，大部分感應器位置可以透過nocs_pos的部分+感應器在模組參考點的偏移量

# 方便存取 yaml 的 Class
class YamlUpdater:
    def __init__(self):
        pass
    # 輔助函數：安全地獲取巢狀字典中的值
    def _get_nested_value(self, data, keys):
        for key in keys:
            if isinstance(data, dict) and key in data:
                data = data[key]
            else:
                return None
        return data
    # 輔助函數：安全地設定巢狀字典中的值
    def _set_nested_value(self, data, keys, value):
        current = data
        for i, key in enumerate(keys):
            if i == len(keys) - 1: # 找到目標鍵
                if isinstance(current, dict):
                    current[key] = value
                    return True
                else:
                    return False # 父級不是字典
            if not isinstance(current, dict) or key not in current:
                return False # 路徑不存在
            current = current[key]
        return False # 不應該走到這裡

class Profile_updater():
    # Initialize the parameter set
    def __init__(self):
        self.params = Profile_params()
    # 1. 更新 2wd_properties.urdf.xacro 檔案
    def update_2wd_properties(self, file_location: str):
        # 更新 2wd_properties.urdf.xacro 檔案
        try:
            # 註冊 xacro 命名空間，以便正確解析 XML
            # 注意：對於 find 方法，我們需要傳遞命名空間字典
            namespaces = {'xacro': 'http://ros.org/wiki/xacro'}
            ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')

            # 解析 XML 檔案
            tree = ET.parse(file_location)
            root = tree.getroot()
            fail = False

            # 1. 尋找名稱為 "laser_pose" 的 xacro:property 元素
            # 使用 find 而不是 findall，因為我們期望只有一個
            laser_pose_element = root.find(".//xacro:property[@name='laser_pose']", namespaces=namespaces)
            if laser_pose_element is not None:
                # 尋找 origin 子元素
                origin_element = laser_pose_element.find('origin')
                if origin_element is not None:
                    # 更新 xyz 屬性
                    new_xyz_value = f"{self.params.laser_pos_x} {self.params.laser_pos_y} {self.params.laser_pos_z}"
                    new_rpy_value = f"{self.params.laser_pos_r} {self.params.laser_pos_p} {self.params.laser_pos_yw}"
                    origin_element.set('xyz', str(new_xyz_value))
                    origin_element.set('rpy', str(new_rpy_value))
                    print(f"[UPDATER] laser_pose updated: xyz: {new_xyz_value}, rpy: {new_rpy_value}")
                else:
                    fail = True
                    print("[UPDATER] [ERROR] No \"origin\" element in the laser_pose props!")
            else:
                fail = True
                print("[UPDATER] [ERROR] No \"laser_pose\" element in the file!")
            # 建立一個映射字典，將 base_param 名稱對應到 self.params 的屬性
            param_value_map = {
                # 修改底盤長寬高
                "base_length": self.params.robot_size_x,
                "base_width": self.params.robot_size_y,
                "base_height": self.params.robot_size_z,
                "wheel_radius": self.params.wheel_radius,
                "wheel_width": self.params.wheel_thick,
                "wheel_pos_x": self.params.wheel_pos_x,
                "wheel_pos_y": self.params.wheel_separate,
                "wheel_pos_z": self.params.wheel_pos_z,
            }
            for base_param, new_value in param_value_map.items():
                base_param_e = root.find(f".//xacro:property[@name='{base_param}']", namespaces=namespaces)
                if base_param_e is not None:
                    # 修改 'value' 屬性
                    base_param_e.set("value", str(new_value))
                    print(f"[UPDATER] {base_param} updated: {new_value}")
                else:
                    fail = True
                    print(f"[UPDATER] [ERROR] No \"{base_param}\" element in the file!")
            # 最後都沒有問題就寫入檔案
            if not (fail):
                # 將修改寫回檔案
                # pretty_print 函數可以讓輸出更易讀，但 ElementTree 內建的 write 預設不會格式化
                # 如果需要格式化，需要額外處理，這裡只進行簡單的寫入
                tree.write(file_location, encoding="utf-8", xml_declaration=True)
                print(f"[UPDATER] Writed: {file_location}")
        except Exception as e:
            print(f"[UPDATER] [ERROR] Got error!, {e}")

    # 2. 更新 imu_properties.urdf.xacro 檔案
    def update_imu_properties(self, file_location: str):
        # 更新 2wd_properties.urdf.xacro 檔案
        try:
            # 註冊 xacro 命名空間，以便正確解析 XML
            # 注意：對於 find 方法，我們需要傳遞命名空間字典
            namespaces = {'xacro': 'http://ros.org/wiki/xacro'}
            ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')

            # 解析 XML 檔案
            tree = ET.parse(file_location)
            root = tree.getroot()
            fail = False

            # 1. 尋找名稱為 "imu_to_base_line" 的 joint 元素
            # 使用 find 而不是 findall，因為我們期望只有一個
            imu_element = root.find(".//joint[@name='imu_to_base_link']", namespaces=namespaces)
            if imu_element is not None:
                # 尋找 origin 子元素
                origin_element = imu_element.find('origin')
                if origin_element is not None:
                    # 更新 xyz 屬性
                    new_xyz_value = f"{self.params.imu_pos_x} {self.params.imu_pos_y} {self.params.imu_pos_z}"
                    new_rpy_value = f"{self.params.imu_pos_r} {self.params.imu_pos_p} {self.params.imu_pos_yw}"
                    origin_element.set('xyz', str(new_xyz_value))
                    origin_element.set('rpy', str(new_rpy_value))
                    print(f"[UPDATER] imu_to_base_link updated: xyz: {new_xyz_value}, rpy: {new_rpy_value}")
                else:
                    fail = True
                    print("[UPDATER] [ERROR] No \"origin\" element in the imu_to_base_link props!")
            else:
                fail = True
                print("[UPDATER] [ERROR] No \"imu_to_base_link\" element in the file!")
            # 最後都沒有問題就寫入檔案
            if not (fail):
                # 將修改寫回檔案
                tree.write(file_location, encoding="utf-8", xml_declaration=True)
                print(f"[UPDATER] Writed: {file_location}")
        except Exception as e:
            print(f"[UPDATER] [ERROR] Got error!, {e}")

    # 更新導航參數
    def update_navigation_param(self, yaml_filepath: str):
        if not os.path.exists(yaml_filepath):
            print(f"[UPDATER] [ERROR] No such file or directory: {yaml_filepath}")
            return

        try:
            yaml_updater = YamlUpdater()

            with open(yaml_filepath, 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)

            # 定義需要更新的參數及其對應的 YAML 路徑和 self.params 屬性
            updates = [
                {
                    "path": ['global_costmap', 'global_costmap', 'ros__parameters', 'robot_radius'],
                    "value": self.params.robot_radius,
                    "name": "robot_radius (global_costmap)"
                },
                {
                    "path": ['local_costmap', 'local_costmap', 'ros__parameters', 'robot_radius'],
                    "value": self.params.robot_radius,
                    "name": "robot_radius (local_costmap)"
                },
                {
                    "path": ['global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'],
                    "value": self.params.inflation_radius,
                    "name": "inflation_radius (global_costmap)"
                },
                {
                    "path": ['local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'],
                    "value": self.params.inflation_radius,
                    "name": "inflation_radius (local_costmap)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'min_vel_x'],
                    "value": self.params.min_linear_vel,
                    "name": "min_vel_x (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'max_vel_x'],
                    "value": self.params.max_linear_vel,
                    "name": "max_vel_x (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'min_speed_xy'],
                    "value": self.params.min_linear_vel,
                    "name": "min_speed_xy (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'max_speed_xy'],
                    "value": self.params.max_linear_vel,
                    "name": "max_speed_xy (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'min_speed_theta'],
                    "value": self.params.min_angular_vel,
                    "name": "min_speed_theta (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'FollowPath', 'max_vel_theta'],
                    "value": self.params.max_angular_vel,
                    "name": "max_vel_theta (FollowPath)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'general_goal_checker', 'xy_goal_tolerance'],
                    "value": self.params.xy_goal_tolerance,
                    "name": "xy_goal_tolerance (general_goal_checker)"
                },
                {
                    "path": ['controller_server', 'ros__parameters', 'general_goal_checker', 'yaw_goal_tolerance'],
                    "value": self.params.yaw_goal_tolerance,
                    "name": "yaw_goal_tolerance (general_goal_checker)"
                },
                {
                    "path": ['velocity_smoother', 'ros__parameters', 'max_velocity'],
                    "value": [self.params.max_linear_vel, 0.0, self.params.max_angular_vel],
                    "name": "max_velocity (velocity_smoother)"
                },
                {
                    "path": ['velocity_smoother', 'ros__parameters', 'min_velocity'],
                    "value": [self.params.max_linear_vel * -1.0, 0.0, self.params.max_angular_vel * -1.0],
                    "name": "min_velocity (velocity_smoother)"
                },
                {
                    "path": ['velocity_smoother', 'ros__parameters', 'deadband_velocity'],
                    "value": [0.0, 0.0, 0.0],
                    "name": "deadband_velocity (velocity_smoother)"
                },
            ]

            fail = True

            for update in updates:
                path = update["path"]
                value = update["value"]
                name = update["name"]

                # 嘗試設定值
                if yaml_updater._set_nested_value(yaml_data, path, value):
                    print(f"[UPDATER] {name} updated: {value}")
                else:
                    print(f"[UPDATER] [ERROR] Failed to update \"{name}\" param. Path not found or invalid structure.")
                    fail = False

            if fail:
                # 最後寫入檔案
                with open(yaml_filepath, 'w', encoding='utf-8') as f:
                    yaml.safe_dump(yaml_data, f, indent=2, allow_unicode=True)
                print(f"[UPDATER] Writed: {yaml_filepath}")
            else:
                print(f"[UPDATER] [WARNING] Not all parameters were updated. File might not be written if critical updates failed.")

        except Exception as e:
            print(f"[UPDATER] [ERROR] Got error: {e}")

    # 執行colcon build
    def colcon_build(self, workspace_path: str, packages: list):
        # Construct the colcon build command
        command = ["colcon", "build"]
        if packages:
            command.extend(["--packages-select"] + packages)

        print(f"[UPDATER] Attempting to run command: {' '.join(command)}")
        print(f"[UPDATER] In directory: {os.path.abspath(workspace_path)}\n")

        process = None # Initialize process outside try block

        try:
            # Start the subprocess in the specified working directory
            # text=True decodes stdout/stderr as text
            # bufsize=1 ensures line-buffering for live output
            process = subprocess.Popen(
                command,
                cwd=workspace_path,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1  # Line-buffered output
            )

            # Read and print stdout live
            print("[UPDATER] Output: ")
            for line in process.stdout:
                print(line, end='') # end='' to prevent double newlines

            # Wait for the process to complete and get return code and any remaining stderr
            # communicate() waits for the process to terminate
            stdout_residual, stderr_output = process.communicate()

            # Print any remaining stdout if not already consumed by the loop
            if stdout_residual:
                print(stdout_residual, end='')

            print("\n[UPDATER] Finished!")
            print(f"[UPDATER] Return Code: {process.returncode}")

            # Check return code for errors
            if process.returncode != 0:
                print("\n[UPDATER] [ERROR] Process returns error:")
                print(stderr_output, end='') # Print stderr if command failed
            else:
                print("\n[UPDATER] Command executed successfully.")

        except Exception as e:
            print(f"[UPDATER] [ERROR] Got error: {e}")
            if process and process.stderr:
                stderr_output_on_exception = process.stderr.read()
                if stderr_output_on_exception:
                    print("[UPDATER] [ERROR] Got std error: ")
                    print(stderr_output_on_exception, end='')

    # Writes the given map name to the `map_filename.conf` file located in
    # `linorobot2/linorobot2_navigation/config/`.
    def write_map_filename(self, file_path: str):
        map_filename = self.params.map_filename
        try:
            with open(file_path, 'w') as f:
                f.write(map_filename.strip()) # Use strip() to remove any leading/trailing whitespace
            print(f"[UPDATER] Successfully wrote map name '{map_filename.strip()}' to: {file_path}")
        except Exception as e:
            print(f"[UPDATER] [ERROR] Got error: {e}")


# 程式進入點
if __name__ == "__main__":
    # 呼叫函數來更新檔案
    updater = Profile_updater()
    # 更新2WD底盤屬性檔案
    updater.update_2wd_properties("../linorobot2/linorobot2_description/urdf/2wd_properties.urdf.xacro")
    # 更新IMU屬性檔案
    updater.update_imu_properties("../linorobot2/linorobot2_description/urdf/sensors/imu.urdf.xacro")
    # 更新導航屬性檔案
    updater.update_navigation_param("../linorobot2/linorobot2_navigation/config/navigation.yaml")
    # 更新地圖名稱(放置一個檔案到 linorobot_navigation 的 config 下)
    updater.write_map_filename("../linorobot2/linorobot2_navigation/config/map_filename.conf")
    # 最後進行編譯
    updater.colcon_build("../../", ["linorobot2_navigation", "linorobot2_description"])
