from time   import sleep
from os     import path
from ctypes import (cdll, create_string_buffer, Structure, byref,
                    c_uint64, c_uint32, c_float, c_byte)

_cur_dir = path.dirname( path.abspath( __file__ ) )

# コマンドを何秒おきに叩くか
INTERVAL_CMD = 0.005
# グリッパー開閉にかかる時間の目安(ms)
INTERVAL_GRIP = 1000
# 何回コマンド送信に失敗したら諦めるか
LIM_COMMAND = 5
# Dobotの動かし方
PTP_MODE = 1 #PTPMOVJXYZMode

ERR_INDEX = 1 << 29

class DobotAPI():
    def __init__(self, log_api):
        self.is_connected = False
        self.api = cdll.LoadLibrary(_cur_dir + "/libDobotDll.so.1.0.0")
        self.last_cmd = -1
        self.stop_flg = False

        self.log_api = log_api
        self.logger = log_api.logger

    def __send_cmd(self, cmd, *args):
        """コマンド送信を成功するまでループさせ一定回数失敗したらエラーを返す"""
        result = cmd(*args)
        if result == 0:
            return result
        elif result == 1:
            raise ConnectionError('connection error with sending command')
        elif result == 2:
            raise TimeoutError('timeout error with sending command')
        else :
            raise RuntimeError('unknown error with sending command')
    def __queue_cmd(self, cmd, *args):
        """
        コマンドをキューに積む
        送信成功するまでループさせ一定回数失敗したらエラーを返す
        成功したらコマンドのインデックスを返す
        """
        # ストップコマンドが掛かっている時は新しいイベントを受け付けない
        if self.stop_flg is True :
            return -1
        queued_cmd_index = c_uint64(0)
        self.__send_cmd(cmd, *args, 1, byref(queued_cmd_index))
        idx = queued_cmd_index.value
        self.last_cmd = idx
        return self.last_cmd

    # 初期化関係
    def disconnect(self):
        self.api.DisconnectDobot()
        self.is_connected = False
    def connect(self):
        if self.api.SearchDobot(create_string_buffer(1000),  1000) == 0:
            raise ConnectionError('dobot not found')

        port_name = ""
        baud_rate = 115200

        sz_para = create_string_buffer(100)
        sz_para.raw = port_name.encode("utf-8")
        fw_type = create_string_buffer(100)
        version = create_string_buffer(100)
        result  = self.api.ConnectDobot(sz_para,  baud_rate,  fw_type,  version)
        if result == 0 : # No Error
            self.is_connected = True
        elif result == 1:
            raise ConnectionError('connection error with first connecting')
        elif result == 2:
            raise TimeoutError('timeout error with first connecting')
        else :
            raise RuntimeError('unknown error with first connecting')

    def queued_cmd_clear(self): self.__send_cmd(self.api.SetQueuedCmdClear)
    def queued_cmd_start(self): self.__send_cmd(self.api.SetQueuedCmdStartExec)
    def set_home_params(self, pos):
        param = HomeParams()
        param.x = pos["x"]
        param.y = pos["y"]
        param.z = pos["z"]
        param.r = pos["ang"]
        return self.__queue_cmd(self.api.SetHOMEParams, byref(param))
    def set_ptp_joint_prms(self, params):
        param = PTPJointParams()
        param.joint1Velocity = params["vel"][0]
        param.joint2Velocity = params["vel"][1]
        param.joint3Velocity = params["vel"][2]
        param.joint4Velocity = params["vel"][3]
        param.joint1Acceleration = params["acc"][0]
        param.joint2Acceleration = params["acc"][1]
        param.joint3Acceleration = params["acc"][2]
        param.joint4Acceleration = params["acc"][3]
        return self.__queue_cmd(self.api.SetPTPJointParams, byref(param))
    def set_ptp_common_prms(self, params):
        param = PTPCommonParams()
        param.velocityRatio = params["vel"]
        param.accelerationRatio = params["acc"]
        return self.__queue_cmd(self.api.SetPTPCommonParams, byref(param))

    # 情報取得
    def get_cur_cmd_index(self):
        queued_cmd_index = c_uint64(0)
        counter = 0
        while True :
            self.__send_cmd(self.api.GetQueuedCmdCurrentIndex, byref(queued_cmd_index))
            idx = queued_cmd_index.value
            if idx <= self.last_cmd :  break

            counter += 1
            if counter > LIM_COMMAND :  raise TimeoutError('timeout error with getting current command')
            sleep(0.5)
        return idx
    def get_pose(self):
        pose = Pose()
        self.__send_cmd(self.api.GetPose, byref(pose))
        return { "x": pose.x, "y": pose.y, "z": pose.z, "ang": pose.rHead }

    # Dobot 動作
    def reset_home(self):
        cmd = HOMECmd()
        cmd.temp = 0
        return self.__queue_cmd(self.api.SetHOMECmd, byref(cmd))
    def ptp(self, pos):
        cmd = PTPCmd()
        cmd.ptpMode = PTP_MODE
        cmd.x = pos["x"]
        cmd.y = pos["y"]
        cmd.z = pos["z"]
        r = pos["ang"]
        if   r < -180 : r += 360
        elif r >  180 : r -= 360
        cmd.rHead = r
        return self.__queue_cmd(self.api.SetPTPCmd, byref(cmd))

    def open_gripper(self):
        self.__queue_cmd(self.api.SetEndEffectorGripper, 1, 0)
        return self.wait_cmd(INTERVAL_GRIP)
    def close_gripper(self):
        self.__queue_cmd(self.api.SetEndEffectorGripper, 1, 1)
        return self.wait_cmd(INTERVAL_GRIP)
    def stop_pump(self):
        return self.__queue_cmd(self.api.SetEndEffectorSuctionCup, 1, 0)

    def wait_cmd(self, ms):
        cmd = WAITCmd()
        cmd.waitTime = ms
        return self.__queue_cmd(self.api.SetWAITCmd, byref(cmd))
    def force_stop(self):
        self.stop_flg = True
        self.__send_cmd(self.api.SetQueuedCmdForceStopExec)
    def restart(self):
        self.stop_flg = False
    
    # 外部IO
    IO_MODES_LIST = (
        "Invalid",
        "Level_Output",
        "PWM_Output",
        "Level_Input",
        "A/D_Input",
        "Pull-up_Input",
        "Pull-down_Input"
    )
    def config_io(self, address, mode):
        if not mode in DobotAPI.IO_MODES_LIST:
            raise ValueError("mode is invalid.")
        cmd = TagIOMultiplexing()
        cmd.address = address
        cmd.mode = DobotAPI.IO_MODES_LIST.index(mode)
        return self.__queue_cmd(self.api.SetIOMultiplexing, byref(cmd))
    def set_io(self, address, level):
        cmd = TagIODO()
        cmd.address = address
        cmd.level   = level
        return self.__queue_cmd(self.api.SetIODO, byref(cmd))


# コマンド用の構造体群
class HomeParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float) ]
class HOMECmd(Structure):
    _pack_ = 1
    _fields_ = [ ("temp", c_float) ]

class Pose(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("joint1Angle", c_float),
        ("joint2Angle", c_float),
        ("joint3Angle", c_float),
        ("joint4Angle", c_float) ]

class WAITCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("waitTime", c_uint32)
        ]

class PTPJointParams(Structure):
    _fields_ = [
        ("joint1Velocity", c_float),
        ("joint2Velocity", c_float),
        ("joint3Velocity", c_float),
        ("joint4Velocity", c_float),
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float) ]
class PTPCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float),
        ("accelerationRatio", c_float) ]
class PTPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float) ]

class TagIOMultiplexing(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("mode", c_byte)
    ]

class TagIODO(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("level", c_byte)
    ]
