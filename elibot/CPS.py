import socket
import json
import time
import numpy as np
import ast
from elibot.Jodell_gripper import Gripper
from scipy.spatial.transform import Rotation as R


class CPSClient:
    def __init__(self, ip, port=8055):
        self.ip = ip
        self.port = port
        self.sock = None
        self.gripper = Gripper()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            return True
        except Exception as e:
            if self.sock:
                self.sock.close()
            self.sock = None
            return False

    def send_power_on_cmd(self):
        method = "set_robot_power_status"
        params = {"status": 1}
        ret, result, id = self.sendCMD(method, params)
        return result

    def send_power_off_cmd(self):
        method = "set_robot_power_status"
        params = {"status": 0}
        ret, result, id = self.sendCMD(method, params)
        return result

    def get_power_status(self):
        method = "get_robot_power_status"
        ret, result, id = self.sendCMD(method)
        if result == "2":
            return True
        else:
            return False

    def power_on(self):
        if not self.get_power_status():
            self.send_power_on_cmd()
            start_time = time.time()
            while True:
                result = self.get_power_status()
                if result:
                    print("上电成功！")
                    break
                elif not result and time.time() - start_time >= 30:
                    print("已尝试30s，上电失败")
                    break
                else:
                    print("上电中...")
            return result
        else:
            return True

    def power_off(self):
        if self.get_power_status():
            self.send_power_off_cmd()
            start_time = time.time()
            while True:
                result = self.get_power_status()
                if not result:
                    print("下电成功！")
                    break
                elif result and time.time() - start_time >= 30:
                    print("已尝试30s，下电失败")
                    break
                else:
                    print("下电中...")
            return not result
        else:
            return True

    def clearAlarm(self):
        ret, result, id = self.sendCMD("clearAlarm")
        print("清 除 报 警 等 待 抱 闸 释 放")
        # print("ret = ", ret, " ", "id = ", id)
        if (ret == True):
            brakeopen = []
            b_sum = 0
            # brakeopen 为6个轴抱闸打开情况，打开为 1，不然为 0，全部打开为[1, 1, 1, 1, 1, 1]
            while b_sum != 6:
                # 获取伺服抱闸打开情况
                suc, result, id = self.sendCMD("get_servo_brake_off_status")
                brakeopen = json.loads(result)
                b_sum = 0
                for d in brakeopen:
                    b_sum = b_sum + d
                time.sleep(0.1)
                # 等待 6 个轴抱闸全部打开
            print("已释放：result = ", result)
            time.sleep(0.1)

        else:
            print("err_msg = ", result["message"])

    def get_servo_status(self):
        method = "getServoStatus"
        ret, result, id = self.sendCMD(method)
        if result == "true":
            return True
        else:
            return False

    def send_set_servo_cmd(self, status=1):  # 0为关，1为开
        method = "set_servo_status"
        params = {"status": status}
        ret, result, id = self.sendCMD(method, params)
        if result == "true":
            return True
        else:
            return False

    def setMotorStatus(self):
        ret, result, id = self.sendCMD("getMotorStatus")
        print("获取同步状态")
        if result == "false":
            ret1, result1, id = self.sendCMD("syncMotorStatus")
            # print("ret = ", ret1, " ", "id = ", id)
            if (ret1 == True):
                print("同步成功，result = ", result1)
                return True
            else:
                print("err_msg = ", result1["message"])
                return False
        else:
            print(result)
            return True

    def set_servo(self, status=1):
        arm_status = self.get_servo_status()
        self.send_set_servo_cmd(status)
        time.sleep(0.5)
        if status == 1:
            if not arm_status:
                self.send_set_servo_cmd(status)
                start_time = time.time()
                while True:
                    result = self.get_servo_status()
                    if result:
                        print("使能成功！")
                        break
                    elif not result and time.time() - start_time >= 30:
                        print("已尝试30s，使能失败")
                        break
                    else:
                        print("使能中...")
                return result
            else:
                return True
        elif status == 0:
            if arm_status:
                self.send_set_servo_cmd(status)
                start_time = time.time()
                while True:
                    result = self.get_servo_status()
                    if not result:
                        print("解除使能成功！")
                        break
                    elif result and time.time() - start_time >= 30:
                        print("已尝试30s，解除使能失败")
                        break
                    else:
                        print("正在解除使能中...")
                return not result
            else:
                return True
        else:
            print("不合理的使能参数")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = None

    def sendCMD(self, cmd, params=None, id=1):
        if not params:
            params = {}
        sendStr = json.dumps({
            "jsonrpc": "2.0",
            "method": cmd,
            "params": params,
            "id": id
        }) + "\n"
        try:
            self.sock.sendall(sendStr.encode('utf-8'))
            ret = self.sock.recv(1024)
            jdata = json.loads(ret.decode('utf-8'))
            if "result" in jdata:
                return True, jdata["result"], jdata["id"]
            elif "error" in jdata:
                return False, jdata["error"], jdata["id"]
            else:
                return False, None, None
        except Exception as e:
            return False, str(e), None

    def getJointPos(self):
        suc, joint_pose, id = self.sendCMD("get_joint_pos")
        if isinstance(joint_pose, str):
            joint_pose = json.loads(joint_pose)
        return joint_pose

    def getTCPPose(self, unit_type=0):
        params = {"unit_type": unit_type}
        suc, result_pose, _ = self.sendCMD("getTcpPose", params=params)
        if isinstance(result_pose, str):
            result_pose = json.loads(result_pose)
        return result_pose

    def moveByJoint(self, target_joint, speed=10, block=True):
        if target_joint[0] <= 120:
            print(target_joint[0])
            raise Exception("Joint1 over limit!", target_joint[0])
        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": target_joint, "speed": speed})
        if suc:
            print(f"Move command sent: Target position {target_joint} with speed {speed}")
            if block:
                while True:
                    _, state, _ = self.sendCMD("getRobotState")
                    if state == '0':  # Assuming '0' means the robot has stopped
                        print("Robot reached the target position.")
                        break
                    time.sleep(0.1)
        else:
            raise Exception("Failed to move robot to the target joint.")

    def moveByJoint_right(self, target_joint, speed=10, block=True):
        if target_joint[0] >= -120:
            raise Exception("Joint1 over limit!.", target_joint[0])
        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": target_joint, "speed": speed})
        if suc:
            print(f"Move command sent: Target position {target_joint} with speed {speed}")
            if block:
                while True:
                    _, state, _ = self.sendCMD("getRobotState")
                    if state == '0':  # Assuming '0' means the robot has stopped
                        print("Robot reached the target position.")
                        break
                    time.sleep(0.1)
        else:
            raise Exception("Failed to move robot to the target joint.")

    def moveBySpeedl(self, speed_l, acc, arot, t, id=1):
        params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
        return self.sendCMD("moveBySpeedl", params, id)

    def inverseKinematic(self, targetPose, unit_type=0):
        referencePos = self.getJointPos()
        targetPose = targetPose.tolist() if isinstance(targetPose, np.ndarray) else targetPose
        params = {"targetPose": targetPose, "referencePos": referencePos, "unit_type": unit_type}
        _, iK_joint, _ = self.sendCMD("inverseKinematic", params)
        if isinstance(iK_joint, str):
            iK_joint = json.loads(iK_joint)
        return iK_joint

    def move_robot(self, target_pose, speed=10,block=True):
        iK_joint = self.inverseKinematic(target_pose)
        current_pos = self.getJointPos()

        # 对比每个对应的关节角差值
        for idx, (ik_val, pos_val) in enumerate(zip(iK_joint, current_pos)):
            diff = abs(ik_val - pos_val)
            if diff > 90:
                raise ValueError(f"第 {idx + 1} 个关节的角度差值 {diff} 超过 90 度！")
        self.moveByJoint(iK_joint, speed=speed,block=block)
        return

    def move_right_robot(self, target_pose, speed=10,block=True):
        iK_joint = self.inverseKinematic(target_pose)
        self.moveByJoint_right(iK_joint, speed=speed,block=block)
        return

    def alignZAxis(self):
        # Get the current TCP pose
        current_pose = self.getTCPPose()
        current_pose[3:] = [180, 0, 0]
        target_pose = current_pose

        # Calculate the joint positions needed to achieve this pose
        iK_joint = self.inverseKinematic(target_pose)

        # Move to the calculated joint positions
        self.moveByJoint(iK_joint, speed=10)

    # Gripper #drg
    def open_tci(self):
        suc, result, _ = self.sendCMD("open_tci")
        return result

    def set_tci(self, baud_rate=9600, bits=8, event="N", stop=1):
        suc, result, _ = self.sendCMD("setopt_tci",
                                      {"baud_rate": baud_rate, "bits": bits, "event": event, "stop": stop})
        return result

    def recv_tci(self, count=100, hex=1, timeout=200):
        suc, result, _ = self.sendCMD("recv_tci", {"count": count, "hex": hex, "timeout": timeout})
        if type(result) == dict:
            print(result)
            return True, None, None

        data = json.loads(result)
        # 提取 "buf" 的值并赋给变量 received
        result = data["result"]
        size = data["size"]
        received = data['buf']
        return suc, result, size, received

    def send_tci(self, send_buf, hex=1):

        suc, result, _ = self.sendCMD("send_tci", {"send_buf": send_buf, "hex": hex})
        return suc, result

    def flush_tci(self):
        suc, result, _ = self.sendCMD("flush_tci")
        return result

    def close_tci(self):
        suc, result, _ = self.sendCMD("close_tci")
        return result

    def connect_gripper(self):
        self.open_tci()

        result = self.set_tci(self.gripper.serial_params["baud_rate"], self.gripper.serial_params["data_bits"],
                              self.gripper.serial_params["event"], self.gripper.serial_params["stop_bits"])
        # 清空寄存器
        self.flush_tci()
        # 发送激活指令
        suc, result = self.send_tci(self.gripper.activate_request(), 1)

        # 接收回复
        suc, result, size, buf = self.recv_tci(100, 1, 500)
        suc, result = self.send_tci(self.gripper.enable_gripper(), 1)
        suc, result, size, buf = self.recv_tci(100, 1, 500)

        self.flush_tci()
        function_code, received = self.gripper.decode_response(buf)

        while True:
            activate_state, error_state, move_state, current_position = self.read_gripper_state()
            if activate_state == 3:
                print("使能成功")
                break
            if error_state != 0:
                print("使能失败！错误码：", error_state)
                break
        self.open_gripper()
        return

    def run_gripper(self, target_position=0x00, force=255, speed=255):
        # 首先清空缓存
        self.flush_tci()
        # 发送打开夹爪指令
        self.send_tci(self.gripper.run_gripper(target_position, force, speed))
        # 接收指令回复
        suc, result, size, buf = self.recv_tci(100, 1, 500)
        while True:
            _, error_state, move_state, current_position = self.read_gripper_state()
            if error_state != 0:
                print("error occurred! error state:", error_state)
                break
            if not move_state:
                print(f"运动已停止当前位置为：{current_position}")
                break
            else:
                continue
        return

    def open_gripper(self):
        self.run_gripper(00, 255, 255)
        return

    def close_gripper(self):
        self.run_gripper(255, 255, 255)
        return

    def read_gripper_state(self):
        self.send_tci(self.gripper.read_gripper_state(3))
        suc, result, size, buf = self.recv_tci()
        function_code, received = self.gripper.decode_response(buf)
        data_length = received[0]
        command = received[1:]
        activate_state, move_state, hand_state, current_position, error_state, current_speed, current_force = self.gripper.decode_state_data(
            data_length, command)
        return activate_state, error_state, move_state, current_position


def desire_left_pose(rpy_array=None):
    # 计算 inv(rpy = (65, 0, 10)) @ rpy = (180, 0, 0)
    if rpy_array is None:
        rpy_array = [180, 0, 180]
    rpy_inv_65_0_10 = R.from_euler('xyz', [65, 4, 12], degrees=True).inv()
    rpy = R.from_euler('xyz', rpy_array, degrees=True)

    # 计算旋转矩阵
    result_inv = rpy_inv_65_0_10.as_matrix() @ rpy.as_matrix()
    rpy_angles = R.from_matrix(result_inv).as_euler('xyz', degrees=True)
    return rpy_angles


def desire_right_pose(rpy_array=None):
    # 计算 inv(rpy = (65, 0, 10)) @ rpy = (180, 0, 0)
    if rpy_array is None:
        rpy_array = [180, 0, 180]
    rpy_inv_65_0_10 = R.from_euler('xyz', [65.33430565, -4.20854252,-9.07946747], degrees=True).inv()
    rpy = R.from_euler('xyz', rpy_array, degrees=True)

    # 计算旋转矩阵
    result_inv = rpy_inv_65_0_10.as_matrix() @ rpy.as_matrix()
    rpy_angles = R.from_matrix(result_inv).as_euler('xyz', degrees=True)
    return rpy_angles


if __name__ == "__main__":
    robot_ip = "192.168.188.200"
    controller = CPSClient(robot_ip)

    if controller.connect():
        print(controller.get_power_status())
        print(controller.get_servo_status())
        result = controller.power_on()
        print(result)
        controller.clearAlarm()
        controller.setMotorStatus()
        print(controller.set_servo(1))
        pose = controller.getTCPPose()
        # joint =controller.getJointPos()
        # print("Joint : ",joint)
        #
        # print(controller.power_off())

        formatted_tcp_pos = [round(pos, 2) for pos in controller.getTCPPose()]
        print("pos :", formatted_tcp_pos)
        formatted_joint_pos = [round(pos, 2) for pos in controller.getJointPos()]
        print("joint :", formatted_joint_pos)
        # # controller.moveByJoint_right([-201.58, -120.88, -114.04, -50.78, -71.61, -116.05])
        # # controller.moveByJoint([174.65, -7.11, 39.66, 65.99, -64.49, 178.81])
        # controller.connect_gripper()
        # controller.run_gripper(255)

        # controller.power_off()
        # time.sleep(3)
        # controller.run_gripper(255,force=255,speed=255)
        #
        # _, error_state, move_state, current_position = controller.read_gripper_state()
        # print("当前位置：", current_position)
        # exit()
        # pose = controller.getJointPos()
        # print(pose)
        # pose[2] = pose[2] + 10
        # rpy_angles = desire_left_pose(rpy_array=[180,0,180]) # left 垂直向下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_left_pose(rpy_array=[0, -90,0])  # left 水平向前
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)

        # rpy_angles = desire_left_pose(rpy_array=[-90,0,90]) # left 水平向前 相机朝右
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_left_pose(rpy_array=[-180, 90, 0])  # left 水平向前 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_left_pose(rpy_array=[-90, 90, 0])  # left 水平向右 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_left_pose(rpy_array=[-90, -90, 0])  # left 水平向右 相机朝上
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)_array=[0,-120
        # rpy_angles = desire_left_pose(rpy_array=[0,-120,0]) # left 斜向前
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[180, 0, 180])  # right 垂直向下
        # pose[3:6] = rpy_angles
        #
        # controller.move_right_robot(pose)

        # rpy_angles = desire_right_pose(rpy_array=[180, -90, 0])  # right 水平向前
        # pose[3:6] = rpy_angles
        # controller.move_right_robot(pose)
        # exit()

        # rpy_angles = desire_right_pose(rpy_array=[-90, 0, 0])  # right 水平向左
        # pose[3:6] = rpy_angles
        # controller.move_right_robot(pose)
        # matrix = R.from_euler('xyz', [65.33430565, -4.20854252,-9.07946747], degrees=True).as_matrix()

        # offset = np.array([-0, 0, 20]) @ matrix
        # pose[:3] = pose[:3] + offset
        # print("Moving down to grab the object...")
        # controller.move_right_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[-90,0,180]) # left 水平向左 相机朝前
        # pose[3:6] = rpy_angles
        # controller.move_right_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[-180, 90, 0])  # left 水平向前 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_right_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[-90, 90, 0])  # left 水平向右 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_right_robot(pose)
