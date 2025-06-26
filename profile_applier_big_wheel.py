import xml.etree.ElementTree as ET

class Profile_params():
    # NOCS position (Use in future)
    nocs_pos_x = 0.0
    nocs_pos_y = 0.0
    nocs_pos_z = 0.0
    nocs_pos_r = 0.0
    nocs_pos_p = 0.0
    nocs_pos_y = 0.0
    # Laser position
    laser_pos_x = 0.0
    laser_pos_y = 0.0
    laser_pos_z = 0.0
    laser_pos_r = 0.0
    laser_pos_p = 0.0
    laser_pos_y = 0.0
    # IMU position
    imu_pos_x = 0.0
    imu_pos_y = 0.0
    imu_pos_z = 0.0
    imu_pos_r = 0.0
    imu_pos_p = 0.0
    imu_pos_y = 0.0
    # Robot size
    robot_size_x = 0.0
    robot_size_y = 0.0
    robot_size_z = 0.0
    # Wheel description
    wheel_pos_x = 0.0
    wheel_pos_z = 0.0
    wheel_separate = 0.0
    wheel_diameter = 0.0
    wheel_thick = 0.0

class Update_profile():

    def update_laser_pose(filepath, laser_x, laser_y, laser_z):
        """
        更新 URDF XACRO 檔案中 laser_pose 的 xyz 座標。

        Args:
            filepath (str): URDF XACRO 檔案的路徑。
            laser_x (float): 新的 X 座標值。
            laser_y (float): 新的 Y 座標值。
            laser_z (float): 新的 Z 座標值。
        """
        try:
            # 註冊 xacro 命名空間，以便正確解析 XML
            # 注意：對於 find 方法，我們需要傳遞命名空間字典
            namespaces = {'xacro': 'http://ros.org/wiki/xacro'}
            ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')

            # 解析 XML 檔案
            tree = ET.parse(filepath)
            root = tree.getroot()

            # 尋找名稱為 "laser_pose" 的 xacro:property 元素
            # 使用 find 而不是 findall，因為我們期望只有一個
            laser_pose_element = root.find(".//xacro:property[@name='laser_pose']", namespaces=namespaces)

            if laser_pose_element is not None:
                # 尋找 origin 子元素
                origin_element = laser_pose_element.find('origin')

                if origin_element is not None:
                    # 更新 xyz 屬性
                    new_xyz_value = f"{laser_x} {laser_y} {laser_z}"
                    origin_element.set('xyz', new_xyz_value)
                    print(f"已成功更新 laser_pose 的 xyz 座標為: {new_xyz_value}")

                    # 將修改寫回檔案
                    # pretty_print 函數可以讓輸出更易讀，但 ElementTree 內建的 write 預設不會格式化
                    # 如果需要格式化，需要額外處理，這裡只進行簡單的寫入
                    tree.write(filepath, encoding="utf-8", xml_declaration=True)
                    print(f"檔案 '{filepath}' 已成功更新。")
                else:
                    print("錯誤：在 laser_pose 屬性中找不到 'origin' 元素。")
            else:
                print("錯誤：在檔案中找不到名稱為 'laser_pose' 的 'xacro:property' 元素。")

        except FileNotFoundError:
            print(f"錯誤：找不到檔案 '{filepath}'。請檢查路徑。")
        except ET.ParseError as e:
            print(f"錯誤：解析 XML 檔案時出錯：{e}")
        except Exception as e:
            print(f"發生未知錯誤：{e}")

# 範例用法：
if __name__ == "__main__":
    # 指定您的 URDF XACRO 檔案路徑
    file_to_edit = '../linorobot2/linorobot2_description/urdf/2wd_properties.urdf.xacro'

    # 指定新的座標值
    new_laser_x = 0.77
    new_laser_y = 0.66
    new_laser_z = 0.55

    # 呼叫函數來更新檔案
    update_laser_pose(file_to_edit, new_laser_x, new_laser_y, new_laser_z)

    # 您可以嘗試使用不同的值再次呼叫：
    # print("\n--- 再次更新為不同值 ---")
    # update_laser_pose(file_to_edit, 0.6, 0.1, 0.4)
