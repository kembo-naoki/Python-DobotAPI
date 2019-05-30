from time   import sleep
from os     import path
from ctypes import (cdll, create_string_buffer, Structure, byref,
                    c_uint64, c_uint32, c_float, c_byte)

_cur_dir = path.dirname( path.abspath( __file__ ) )

class DobotAPI():
    """
    Attributes
    ----------
    INTERVAL_CMD : float
        コマンドの送信間隔(秒)
    INTERVAL_GRIP : int
        グリッパーの開閉のための待ち時間(ms)
    LIM_COMMAND : int
        コマンド送信失敗時のリトライ回数
    PTP_MODE : int
    """

    INTERVAL_CMD = 0.005
    INTERVAL_GRIP = 1000
    LIM_COMMAND = 5
    PTP_MODE = 1 #PTPMOVJXYZMode

    def __init__(self, logger=None):
        """
        Parameters
        ----------
        logger : optional
        """
        self.is_connected = False
        self.api = cdll.LoadLibrary(_cur_dir + "/libDobotDll.so.1.0.0")
        self.last_cmd = -1
        self.stop_flg = False

        if logger is not None: self.logger = logger

    def __send_cmd(self, cmd, *args):
        """
        コマンドを送信する。`LIM_COMMAND`回までリトライする。

        Parameters
        ----------
        cmd : function
            送信したいコマンドの関数
        *args
            上記コマンドへの引数
        """
        result = cmd(*args)
        if result == 0:
            return result
        elif result == 1:
            raise ConnectionError("connection error with sending command")
        elif result == 2:
            raise TimeoutError("timeout error with sending command")
        else :
            raise RuntimeError("unknown error with sending command")
    def __queue_cmd(self, cmd, *args):
        """
        コマンドを Dobot のキューに積む。

        Parameters
        ----------
        cmd : function
            送信したいコマンドの関数
        *args
            上記コマンドへの引数
        
        Returns
        -------
        last_cmd : int
            Dobot のキューインデックス
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
        """ 切断 """
        self.api.DisconnectDobot()
        self.is_connected = False
    def connect(self):
        """ 接続 """
        if self.api.SearchDobot(create_string_buffer(1000),  1000) == 0:
            raise ConnectionError("dobot not found")

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
            raise ConnectionError("connection error with first connecting")
        elif result == 2:
            raise TimeoutError("timeout error with first connecting")
        else :
            raise RuntimeError("unknown error with first connecting")

    def queued_cmd_clear(self):
        """ キューに積まれたコマンドをクリア """
        self.__send_cmd(self.api.SetQueuedCmdClear)
    def queued_cmd_start(self):
        """ キューに積まれたコマンドの実行を開始 """
        self.__send_cmd(self.api.SetQueuedCmdStartExec)
    def set_home_params(self, pos):
        """
        ホームポジションの設定

        Parameters
        ----------
        pos : dict of {str: float}
            "x","y","z"による座標及び"ang"によるグリッパーの角度
        """
        param = HomeParams()
        param.x = pos["x"]
        param.y = pos["y"]
        param.z = pos["z"]
        param.r = pos["ang"]
        return self.__queue_cmd(self.api.SetHOMEParams, byref(param))
    def set_ptp_joint_prms(self, params):
        """
        PTP コマンドのモーター毎の速度加速度の設定

        Parameters
        ----------
        params : dict of {str: list of float}
            "vel"が速度で"acc"が加速度。配列のインデックスは各モーターを表す。
        """
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
        """
        PTP コマンド使用時の速度倍率設定

        Parameters
        ----------
        params : dict of {str: float}
            "vel"が速度で"acc"が加速度。0,1,2 が x,y,z、3 が ang に対応する。
        """
        param = PTPCommonParams()
        param.velocityRatio = params["vel"]
        param.accelerationRatio = params["acc"]
        return self.__queue_cmd(self.api.SetPTPCommonParams, byref(param))

    # 情報取得
    def get_cur_cmd_index(self):
        """
        現在まで実行完了済のキューインデックス

        Returns
        -------
        index : int
        """
        queued_cmd_index = c_uint64(0)
        counter = 0
        while True :
            self.__send_cmd(self.api.GetQueuedCmdCurrentIndex, byref(queued_cmd_index))
            idx = queued_cmd_index.value
            if idx <= self.last_cmd :  break

            counter += 1
            if counter > DobotAPI.LIM_COMMAND :  raise TimeoutError("timeout error with getting current command")
            sleep(0.5)
        return idx
    def get_pose(self):
        """
        現在のグリッパーの座標

        Returns
        -------
        pose : dict of {str: float}
            "x","y","z","ang"
        """
        pose = Pose()
        self.__send_cmd(self.api.GetPose, byref(pose))
        return { "x": pose.x, "y": pose.y, "z": pose.z, "ang": pose.rHead }

    # Dobot 動作
    def reset_home(self):
        """ ホームポジションリセット """
        cmd = HOMECmd()
        cmd.temp = 0
        return self.__queue_cmd(self.api.SetHOMECmd, byref(cmd))
    def ptp(self, pos):
        """ アームの特定座標への移動 """
        cmd = PTPCmd()
        cmd.ptpMode = DobotAPI.PTP_MODE
        cmd.x = pos["x"]
        cmd.y = pos["y"]
        cmd.z = pos["z"]
        r = pos["ang"]
        if   r < -180 : r += 360
        elif r >  180 : r -= 360
        cmd.rHead = r
        return self.__queue_cmd(self.api.SetPTPCmd, byref(cmd))

    def open_gripper(self):
        """ グリッパーを開く """
        self.__queue_cmd(self.api.SetEndEffectorGripper, 1, 0)
        return self.wait_cmd(DobotAPI.INTERVAL_GRIP)
    def close_gripper(self):
        """ グリッパーを閉じる """
        self.__queue_cmd(self.api.SetEndEffectorGripper, 1, 1)
        return self.wait_cmd(DobotAPI.INTERVAL_GRIP)
    def stop_pump(self):
        """ ポンプの停止 """
        return self.__queue_cmd(self.api.SetEndEffectorSuctionCup, 1, 0)

    def wait_cmd(self, ms):
        """
        待機命令

        Parameters
        ----------
        ms : int
            待機時間（ミリ秒）
        """
        cmd = WAITCmd()
        cmd.waitTime = ms
        return self.__queue_cmd(self.api.SetWAITCmd, byref(cmd))
    def force_stop(self):
        """ 強制停止 """
        self.stop_flg = True
        self.__send_cmd(self.api.SetQueuedCmdForceStopExec)
    def restart(self):
        """ 強制停止後の再開命令 """
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
        """ 特定の I/O ポートのモード設定 """
        if not mode in DobotAPI.IO_MODES_LIST:
            raise ValueError("mode is invalid.")
        cmd = TagIOMultiplexing()
        cmd.address = address
        cmd.mode = DobotAPI.IO_MODES_LIST.index(mode)
        return self.__queue_cmd(self.api.SetIOMultiplexing, byref(cmd))
    def set_io(self, address, level):
        """ 特定の I/O ポートの出力設定 """
        cmd = TagIODO()
        cmd.address = address
        cmd.level   = level
        return self.__queue_cmd(self.api.SetIODO, byref(cmd))


""" コマンド用の構造体群 """
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
