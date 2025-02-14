class Gripper:
    def __init__(self,ID=0x09):
        # 串口参数
        self.serial_params = {
            "interface": "RS-485",
            "baud_rate": 115200,
            "event":"N",
            "data_bits": 8,
            "stop_bits": 1,
            "parity": "None",
            "flow_control": "None",
        }
        self.device_address = ID

        self.control_register = 0x03E8  # 控制寄存器地址 低字节为控制寄存器，高字节为无参数指令寄存器

        self.status_register = 0x07D0  # 状态反馈寄存器地址



    def _generate_command_frame(self, data):
        """
        生成Modbus RTU命令帧
        """
        command = [self.device_address] + data
        crc = self._calculate_crc(command)
        return command + crc

    @staticmethod
    def _calculate_crc(data):
        """
        CRC校验码计算
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return [crc & 0xFF, (crc >> 8) & 0xFF]

    def activate_request(self):
        """
        发送激活请求 (rACT=0)，命令帧: 09 10 03 E8 00 01 02 00 00 E5 B8
        :return: 返回需要发送的激活请求命令帧
        """
        function_code = 0x10 #写寄存器功能码
        register_address = self.control_register
        register_number = 0x0001
        bytes_number=0x02
        register_data=0x0000
        command = [
            function_code,         # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,         # 寄存器地址低字节
            (register_number >> 8) & 0xFF,   # 寄存器数量高字节
            register_number & 0xFF,          # 寄存器数量低字节
            bytes_number,                    # 数据字节数
            (register_data >> 8) & 0xFF,     # 寄存器数据高字节
            register_data & 0xFF,            # 寄存器数据低字节
        ]

        command_frame = self._generate_command_frame(command)
        # 将每个整数转换为两位的十六进制字符串，并用空格连接
        hex_string = "".join(f"{x:02X}" for x in command_frame)

        print(hex_string)  # 输出 "48 65 6C 6C 6F"

        return hex_string

    def enable_gripper(self):
        """
         使能夹爪 (rACT=1)，命令帧: ：09 10 03 E8 00 01 02 00 01 24 78
         :return: 返回需要发送的使能请求命令帧
        """
        function_code = 0x10  # 写寄存器功能码
        register_address = self.control_register
        register_number = 0x0001
        bytes_number = 0x02
        register_data = 0x0001
        command = [
            function_code,  # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,  # 寄存器地址低字节
            (register_number >> 8) & 0xFF,  # 寄存器数量高字节
            register_number & 0xFF,  # 寄存器数量低字节
            bytes_number,  # 数据字节数
            (register_data >> 8) & 0xFF,  # 寄存器数据高字节
            register_data & 0xFF,  # 寄存器数据低字节
        ]
        command_frame = self._generate_command_frame(command)
        hex_string = "".join(f"{x:02X}" for x in command_frame)
        print(hex_string)  # 输出 "48 65 6C 6C 6F"
        return hex_string

    def check_data(self, data:int):
        if data < 0x00:
            print("警告：目标位置过小，已设置为 0x00")
            data=0x00
        elif data > 0xFF:
            print("警告：目标位置过大，已设置为 0xFF")
            data=0xFF
        return data


    def run_gripper(self,target_position=0,force=100,velocity=100):
        """
        target_position:夹爪目标位置
        :param target_position:
        :return:
        """
        target_position = self.check_data(target_position)
        force = self.check_data(force)
        velocity = self.check_data(velocity)

        function_code = 0x10  # 写寄存器功能码
        register_address = self.control_register
        register_number = 0x0003
        bytes_number = 0x06
        register_data1 = 0x0009 #激活请求
        register_data2_high=target_position
        register_data2_low=00
        register_data3_high=force
        register_data3_low=velocity

        command = [
            function_code,  # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,  # 寄存器地址低字节
            (register_number >> 8) & 0xFF,  # 寄存器数量高字节
            register_number & 0xFF,  # 寄存器数量低字节
            bytes_number,  # 数据字节数
            (register_data1 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data1 & 0xFF,  # 寄存器数据低字节
            register_data2_high & 0xFF,  # 寄存器数据高字节
            register_data2_low & 0xFF,  # 寄存器数据低字节
            register_data3_high & 0xFF,  # 寄存器数据高字节
            register_data3_low & 0xFF,  # 寄存器数据低字节
        ]
        command_frame = self._generate_command_frame(command)
        hex_string = "".join(f"{x:02X}" for x in command_frame)
        print(hex_string)  # 输出 "48 65 6C 6C 6F"
        return hex_string

    def read_gripper_state(self,register_number=1):
        """
         读取夹爪状态 ，命令帧: ：：09 04 07 D0 00 01 30 0F
         :return: 返回需要发送的请求命令帧
        """
        function_code = 0x04 # 读取输入寄存器
        register_address = self.status_register
        command = [
            function_code,  # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,  # 寄存器地址低字节
            (register_number >> 8) & 0xFF,  # 寄存器数量高字节
            register_number & 0xFF,  # 寄存器数量低字节
        ]
        command_frame = self._generate_command_frame(command)
        hex_string = "".join(f"{x:02X}" for x in command_frame)
        print(hex_string)  # 输出 "48 65 6C 6C 6F"
        return hex_string

    def decode_response(self,command_frame):
        """
        解码Modbus RTU回复帧
        :param command_frame: 回复的命令帧（字节列表）
        :return: 返回解码后的结果，包含从站号，功能码，寄存器地址，寄存器数量，其他内容和CRC校验
        如果function_code是10，写，则content包含写入的寄存器地址4，寄存器数量4
        如果是读取，则content包含：数据字节数：2，独到的内容（长度由前面的字节数决定）
        """
        # 按两位分组
        hex_pairs = [command_frame[i:i + 2] for i in range(0, len(command_frame), 2)]

        # 转换为十六进制数
        hex_values = [int(pair, 16) for pair in hex_pairs]
        # 从站号
        device_address = hex_values[0]

        # 功能码
        function_code = hex_values[1]
        # 其他内容（数据字节，除了CRC）
        content = hex_values[2:-2]  # 去掉CRC校验的最后两个字节

        # CRC 校验码（最后两个字节）
        crc = (hex_values[-2] << 8) | hex_values[-1]

        # 打印输出每个部分
        print(f"从站号: 0x{device_address:02X}")
        print(f"功能码: 0x{function_code:02X}")
        print(f"其他内容: {' '.join([f'0x{byte:02X}' for byte in content])}")
        print(f"CRC 校验码: 0x{crc:04X}")
        return function_code,content

    def decode_state_data(self,data_length,command_frame):
        if len(command_frame) != data_length:
            print("data length error")

        gripper_state=command_frame[1]
        if gripper_state>>2&1==0:
            print("已经停止")
            move_state=False #运动已经停止
        else:
            print("正在前往目标位置")
            move_state=True
        """
        activate_state:激活状态位，第四位和第五位
        3:激活完成
        """
        activate_state=(gripper_state>>4)&3
        """
        夹爪状态：
        0:手指向指定位置移动，未检测到物体
        1:内撑模式已接触到物体
        2:为外夹模式已接触到物体
        3:手指已到达指定的位置，但没有检测到对象
        """
        hand_state=gripper_state>>6
        """读取至少两个寄存器"""
        if data_length > 2:
            """
            当前位置：
            00:完全打开
            FF:完全关闭
            """
            current_position = command_frame[2]
            """
            故障码
            全部是0x00正常，不同位置的1代表不同故障状态
            """
            error_state=command_frame[3]
        else:
            current_position=None
            error_state=None
        """
        读取至少3个寄存器
        """
        if data_length > 4:
            """
            当前速度:00-FF
            """
            current_speed=command_frame[4]
            """
            当前力矩:00-FF
            """
            current_force=command_frame[5]
        else:
            current_speed=None
            current_force=None
        return activate_state,move_state,hand_state,current_position,error_state,current_speed,current_force


    def close_gripper_full_speed_force(self):
        """
        全速全力关闭夹爪
        返回需要发送的串口命令帧
        """
        function_code = 0x10  # 写寄存器功能码
        register_address = self.control_register
        register_number = 0x0003
        bytes_number = 0x06
        register_data1 = 0x0009 #激活请求
        register_data2 = 0xFF00 #03E9位置FF
        register_data3 = 0xFFFF #03EA，低字节 速度，高字节 力

        command = [
            function_code,  # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,  # 寄存器地址低字节
            (register_number >> 8) & 0xFF,  # 寄存器数量高字节
            register_number & 0xFF,  # 寄存器数量低字节
            bytes_number,  # 数据字节数
            (register_data1 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data1 & 0xFF,  # 寄存器数据低字节
            (register_data2 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data2 & 0xFF,  # 寄存器数据低字节
            (register_data3 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data3 & 0xFF,  # 寄存器数据低字节
        ]
        command_frame = self._generate_command_frame(command)
        return command_frame

    def open_gripper_full_speed_force(self):
        """
        全速全力关闭夹爪
        返回需要发送的串口命令帧
        """
        function_code = 0x10  # 写寄存器功能码
        register_address = self.control_register
        register_number = 0x0003
        bytes_number = 0x06
        register_data1 = 0x0009 #激活请求
        register_data2 = 0x0000 #03E9位置FF
        register_data3 = 0xFFFF #03EA，低字节 速度，高字节 力

        command = [
            function_code,  # 功能码
            (register_address >> 8) & 0xFF,  # 寄存器地址高字节
            register_address & 0xFF,  # 寄存器地址低字节
            (register_number >> 8) & 0xFF,  # 寄存器数量高字节
            register_number & 0xFF,  # 寄存器数量低字节
            bytes_number,  # 数据字节数
            (register_data1 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data1 & 0xFF,  # 寄存器数据低字节
            (register_data2 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data2 & 0xFF,  # 寄存器数据低字节
            (register_data3 >> 8) & 0xFF,  # 寄存器数据高字节
            register_data3 & 0xFF,  # 寄存器数据低字节
        ]
        command_frame = self._generate_command_frame(command)
        return command_frame

    def read_current_position(self):
        """
        读取当前位置
        返回读取位置指令帧
        """
        device_address = 0x01  # 设备地址
        function_code = 0x03  # 读取寄存器功能码
        data = [0x03, 0xE9, 0x00, 0x01]  # 动态位置寄存器地址及读取长度
        return self._generate_command_frame(device_address, function_code, data)

    def set_gripper_parameters(self, position, speed, force):
        """
        设置夹爪参数：位置、速度、力
        返回设置命令帧
        """
        device_address = 0x01  # 设备地址
        function_code = 0x10  # 写寄存器功能码
        data = [
            0x00, 0x01,  # 起始地址
            0x03,        # 寄存器数量
            position, speed, force
        ]
        return self._generate_command_frame(device_address, function_code, data)


# 示例使用
def main():
    gripper = Gripper()
    activate_request_commend=gripper.activate_request()
    print("激活夹爪:", " ".join([f"0x{byte:02X}" for byte in activate_request_commend]))
    # open_gripper=gripper.open_gripper_full_speed_force()
    # close_gripper=gripper.close_gripper_full_speed_force()
    # print("打开夹爪:", " ".join([f"0x{byte:02X}" for byte in open_gripper]))
    # print("关闭夹爪:", " ".join([f"0x{byte:02X}" for byte in close_gripper]))
    # run_gripper=gripper.run_gripper()
    # print("打开夹爪:", " ".join([f"0x{byte:02X}" for byte in run_gripper]))
    # response_frame = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x01, 0x80, 0xF1]
    # gripper.decode_response(response_frame)

if __name__ == "__main__":
    main()
