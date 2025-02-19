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
            print("Failed to send move command.")

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

    def move_robot(self, target_pose, speed=10):
        iK_joint = self.inverseKinematic(target_pose)
        self.moveByJoint(iK_joint, speed=speed)
        return 0

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

    def run_gripper(self, target_position=0x00, force=70, speed=40):
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
    rpy_inv_65_0_10 = R.from_euler('xyz', [65, 0, 10], degrees=True).inv()
    rpy = R.from_euler('xyz', rpy_array, degrees=True)

    # 计算旋转矩阵
    result_inv = rpy_inv_65_0_10.as_matrix() @ rpy.as_matrix()
    rpy_angles = R.from_matrix(result_inv).as_euler('xyz', degrees=True)
    return rpy_angles


def desire_right_pose(rpy_array=None):
    # 计算 inv(rpy = (65, 0, 10)) @ rpy = (180, 0, 0)
    if rpy_array is None:
        rpy_array = [180, 0, 180]
    rpy_inv_65_0_10 = R.from_euler('xyz', [-65, 0, -10], degrees=True).inv()
    rpy = R.from_euler('xyz', rpy_array, degrees=True)

    # 计算旋转矩阵
    result_inv = rpy_inv_65_0_10.as_matrix() @ rpy.as_matrix()
    rpy_angles = R.from_matrix(result_inv).as_euler('xyz', degrees=True)
    return rpy_angles


if __name__ == "__main__":
    robot_ip = "192.168.188.201"
    controller = CPSClient(robot_ip)

    if controller.connect():
        pose = controller.getTCPPose()

        formatted_tcp_pos = [round(pos, 2) for pos in controller.getTCPPose()]
        print("pos :",formatted_tcp_pos)
        formatted_joint_pos = [round(pos, 2) for pos in controller.getJointPos()]
        print("joint :",formatted_joint_pos)



        # controller.connect_gripper()
        # controller.run_gripper(0)
        # time.sleep(3)
        # controller.run_gripper(255)
        #
        # _, error_state, move_state, current_position = controller.read_gripper_state()
        # print("当前位置：", current_position)
        # exit()
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
        # controller.move_robot(pose)
        # rpy_angles = desire_left_pose(rpy_array=[0,-120,0]) # left 斜向前
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)

        # rpy_angles = desire_right_pose(rpy_array=[180, 0, 180])  # right 垂直向下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        #
        # rpy_angles = desire_right_pose(rpy_array=[0, -90, 0])  # right 水平向前
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)

        # rpy_angles = desire_right_pose(rpy_array=[-90,0,180]) # left 水平向左 相机朝前
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[-180, 90, 0])  # left 水平向前 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
        # rpy_angles = desire_right_pose(rpy_array=[-90, 90, 0])  # left 水平向右 相机朝下
        # pose[3:6] = rpy_angles
        # controller.move_robot(pose)
