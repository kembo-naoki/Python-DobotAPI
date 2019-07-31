from time   import sleep
from ctypes import (create_string_buffer, Structure, byref, c_float, c_uint32, c_uint64)

from base import (API, CommandModule)
from coordinate import (Coordinate, CartCoord, CartVector, JointCoord, JointVector)
from ptp  import MoveController
from gpio import IOController

class Dobot():
    """
    Attributes
    ----------
    INTERVAL_CMD: float
        コマンドの送信間隔(秒)
    INTERVAL_GRIP: int
        グリッパーの開閉のための待ち時間(ms)
    LIM_COMMAND: int
        コマンド送信失敗時のリトライ回数
    """

    INTERVAL_CMD = 0.005
    INTERVAL_GRIP = 1000
    LIM_COMMAND = 5

    def __init__(self, logger=None):
        """
        Parameters
        ----------
        logger: optional
        """
        self.is_connected = False

        self.queue = QueueController(self)
        self.arm = ArmController(self)
        self.io = IOController(self)

        if logger is not None: self.logger = logger

    # コマンドそのものに関するメソッド
    def send_cmd(self, cmd, *args):
        """
        コマンドを送信する

        Parameters
        ----------
        cmd: function
            送信したいコマンドの関数
        *args
            上記コマンドへの引数
        """
        result = cmd(*args)
        if result == 0:
            return result
        elif result == 1:
            raise DobotConnectionError("connection error with sending command")
        elif result == 2:
            raise DobotTimeout("timeout error with sending command")
        else:
            raise RuntimeError("unknown error with sending command")
    def force_stop(self):
        """
        実行中のコマンドも含めて緊急停止し、キューにコマンドを積めなくなる。
        queue_clear, queue_start, queue_pause のいずれかでキューにコマンドを積めるようになる。
        """
        self.stop_flg = True
        self.send_cmd(API.SetQueuedCmdForceStopExec)

    # 接続／切断
    def connect(self):
        """ 接続 """
        if API.SearchDobot(create_string_buffer(1000),  1000) == 0:
            raise DobotConnectionError("dobot not found")

        port_name = ""
        baud_rate = 115200

        sz_para = create_string_buffer(100)
        sz_para.raw = port_name.encode("utf-8")
        fw_type = create_string_buffer(100)
        version = create_string_buffer(100)
        result  = API.ConnectDobot(sz_para,  baud_rate,  fw_type,  version)
        if result == 0: # No Error
            self.is_connected = True
        elif result == 1:
            raise DobotConnectionError("connection error with first connecting")
        elif result == 2:
            raise DobotTimeout("timeout error with first connecting")
        else:
            raise RuntimeError("unknown error with first connecting")
    def disconnect(self):
        """ 切断 """
        API.DisconnectDobot()
        self.is_connected = False

class QueueController(CommandModule):
    @property
    def last_index(self):
        return self._last_index

    def __init__(self, dobot):
        super().__init__(dobot)
        self.enable = True
        self._last_index = -1

    def send(self, cmd, *args, imm=False):
        """
        コマンドを Dobot のキューに積む。

        Parameters
        ----------
        cmd: function
            送信したいコマンドの関数
        *args
            上記コマンドへの引数
        imm: bool
            True のとき即座に実行する

        Returns
        -------
        last_index: int
            Dobot のキューインデックス
        """
        if not self.enable:
            return -1
        cmd_index = c_uint64(0)
        mode = 0 if imm else 1
        self.dobot.send_cmd(cmd, *args, mode, byref(cmd_index))
        idx = cmd_index.value
        self._last_index = idx
        return self._last_index

    def clear(self):
        """ キューに積まれたコマンドをクリア """
        self.dobot.send_cmd(API.SetQueuedCmdClear)
        self.enable = True

    def start(self):
        """ キューに積まれたコマンドの実行を開始 """
        self.dobot.send_cmd(API.SetQueuedCmdStartExec)
        self.enable = True

    def pause(self):
        """ キューに積まれたコマンドを停止（実行中のコマンドは止まらない） """
        self.dobot.send_cmd(API.SetQueuedCmdStopExec)
        self.enable = False

    def get_current_index(self):
        """
        現在まで実行完了済のキューインデックス

        Returns
        -------
        index: int
        """
        queued_cmd_index = c_uint64(0)
        counter = 0
        while True:
            self.dobot.send_cmd(API.GetQueuedCmdCurrentIndex, byref(queued_cmd_index))
            index = queued_cmd_index.value
            if index <= self._last_index:  break

            counter += 1
            if counter > Dobot.LIM_COMMAND:  raise TimeoutError("timeout error with getting current command")
            sleep(0.5)
        return index

class ArmController(CommandModule):
    """ アーム関連のコマンド群 """
    def __init__(self, dobot):
        self.dobot = dobot
        self.movement = MoveController(dobot)
        self.move_to = self.movement.exec

    def set_home_params(self, coord, *, imm=False):
        """
        ホームポジションの設定

        Parameters
        ----------
        coord: CartCoord
            ホームポジションのデカルト座標
        imm: bool
            True のとき即座に実行する
        """
        param = HomeParams()
        param = coord.infiltrate(param)
        return self.dobot.queue.send(API.SetHOMEParams, byref(param), imm=imm)

    # 情報取得
    def _get_pose(self):
        """
        Returns
        -------
        pose: (CartCoord, JointCoord)
        """
        pose = Pose()
        self.dobot.send_cmd(API.GetPose, byref(pose))
        return (
            CartCoord(pose.x, pose.y, pose.z, pose.rHead, False),
            JointCoord(pose.joint1Angle, pose.joint2Angle,
                       pose.joint3Angle, pose.joint4Angle, False) )

    def get_pose_in_cartesian(self):
        """
        現在のアームの手首部分の座標

        Returns
        -------
        coord: CartCoord
        """
        return self._get_pose()[0]
    def get_pose_in_joint(self):
        """
        現在の各関節の角度

        Returns
        -------
        coord: CartCoord
        """
        return self._get_pose()[1]

    # Dobot 動作
    def reset_home(self, *, imm=False):
        """
        ホームポジションリセット
        imm: bool
            True のとき即座に実行する
        """
        cmd = HOMECmd()
        cmd.temp = 0
        return self.dobot.queue.send(API.SetHOMECmd, byref(cmd), imm=imm)
    def wait(self, ms, *, imm=False):
        """
        待機命令

        Parameters
        ----------
        ms: int
            待機時間（ミリ秒）
        imm: bool
            True のとき即座に実行する
        """
        cmd = WAITCmd()
        cmd.waitTime = ms
        return self.dobot.queue.send(API.SetWAITCmd, byref(cmd), imm=imm)

    def open_gripper(self, *, imm=False):
        """ グリッパーを開く """
        self.dobot.queue.send(API.SetEndEffectorGripper, 1, 0, imm=imm)
        return self.wait(Dobot.INTERVAL_GRIP)
    def close_gripper(self, *, imm=False):
        """ グリッパーを閉じる """
        self.dobot.queue.send(API.SetEndEffectorGripper, 1, 1, imm=imm)
        return self.wait(Dobot.INTERVAL_GRIP)
    def stop_pump(self, *, imm=False):
        """ ポンプの停止 """
        return self.dobot.queue.send(API.SetEndEffectorSuctionCup, 1, 0, imm=imm)

# エラー
class DobotError(Exception):
    """ Dobot から送出されるエラー """
    pass
class DobotTimeout(DobotError, TimeoutError):
    """
    Dobot から送信されるタイムアウトエラー
    通信不安定などによって生じるスクリプト上でのタイムアウトエラーは除く
    """
    pass
class DobotConnectionError(DobotError, ConnectionError):
    """ Dobot との通信に関するエラー """
    pass

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
    _fields_ = [("waitTime", c_uint32)]

