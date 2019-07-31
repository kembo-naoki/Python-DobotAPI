import re
import functools
from os import path
from abc import (ABCMeta, abstractmethod)
from time import (sleep)
from ctypes import (cdll, Structure, byref,
                    create_string_buffer, c_float, c_uint32, c_uint64)
from typing import (SupportsFloat, SupportsInt, Callable,
                    List, Tuple, Dict, Mapping)

from coordinate import (convert_coord, Coordinate,
                        CartCoord, CartVector, JointCoord, JointVector)

_CUR_DIR = path.dirname(path.abspath( __file__ ))
API = cdll.LoadLibrary(_CUR_DIR + "/libDobotDll.so.1.0.0")

class Dobot():
    """ Dobot 本体を表すクラス """

    INTERVAL_CMD = 0.005
    INTERVAL_GRIP = 1000
    LIM_COMMAND = 5

    def __init__(self, logger=None):
        """ インスタンスを作成した時点ではまだ接続はしません。 """
        self.is_connected = False

        self.queue = QueueController(self)
        self.arm = ArmController(self)
        #self.io = IOController(self)

        if logger is not None: self.logger = logger

    # コマンドそのものに関するメソッド
    def send_command(self, cmd:API._FuncPtr, args:tuple = tuple()):
        """ コマンドを送信する """
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
        """ 実行中のコマンドも含めて緊急停止し、キューにコマンドを積めなくなる。

        `Dobot.queue_clear()`, `Dobot.queue_start()`, `Dobot.queue_pause()`のいずれかのメソッドで
        再びキューにコマンドを積めるようになる。
        """
        self.stop_flg = True
        self.send_command(API.SetQueuedCmdForceStopExec)

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

# prepare for command classes
def setting_method(func:Callable):
    """ このメソッドを設定用メソッドとする """
    func._setting_method = True
    @functools.wraps(func)
    def inner(self, *args, **kwargs):
        result = func(self, *args, **kwargs)
        self._progress_of_settings[func.__name__] = True
        return result
    return inner

class _MetaCommand(ABCMeta):
    def __new__(mcls, *args, **kwargs):
        """ クラス定義時に設定用メソッドの一覧を作成 """
        new_cls = super().__new__(mcls, *args, **kwargs)
        new_cls.SETTING_METHODS = tuple(
            f.__name__ for f in new_cls.__dict__.values()
                if isinstance(f, Callable)
                   and getattr(f, "_setting_method", False))
        return new_cls

class DobotCommand(metaclass=_MetaCommand):
    """ Dobot の各機能を表す抽象クラス。 """
    SETTING_METHODS = tuple()

    def __init__(self, dobot:'Dobot'):
        self.dobot = dobot
        self._progress_of_settings = dict([name, False] for name in self.SETTING_METHODS)

    def progress_of_settings(self) -> Dict[str,bool]:
        """ そのコマンドのための設定の完了状況を示すメソッド。

        設定用のメソッド名をキーとし、設定の完了状況を`bool`で示した`dict`が返されます。
        """ 
        return self._progress_of_settings.copy()
    

class QueueController(DobotCommand):
    @property
    def last_index(self):
        return self._last_index

    def __init__(self, dobot:Dobot):
        super().__init__(dobot)
        self._last_index = -1

    def send(self, cmd:API._FuncPtr, args:tuple = tuple(), imm:bool = False) -> int:
        """ コマンドを Dobot のキューに積む。

        キューに積まず即座に実行したい場合は、`imm`を`True`にしてください。
        """
        cmd_index = c_uint64(0)
        mode = 0 if imm else 1
        self.dobot.send_command(cmd, args=(*args, mode, byref(cmd_index)))
        idx = cmd_index.value
        self._last_index = idx
        return self._last_index

    def clear(self):
        """ キューに積まれたコマンドをクリア """
        self.dobot.send_command(API.SetQueuedCmdClear)

    def start(self):
        """ キューに積まれたコマンドの実行を開始 """
        self.dobot.send_command(API.SetQueuedCmdStartExec)

    def pause(self):
        """ キューに積まれたコマンドを停止（実行中のコマンドは止まらない） """
        self.dobot.send_command(API.SetQueuedCmdStopExec)

    def get_current_index(self) -> int:
        """ 現在まで実行完了済のキューインデックス """
        queued_cmd_index = c_uint64(0)
        counter = 0
        while True:
            self.dobot.send_command(API.GetQueuedCmdCurrentIndex,
                                    args=(byref(queued_cmd_index),))
            index = queued_cmd_index.value
            if index <= self._last_index:  break

            counter += 1
            if counter > Dobot.LIM_COMMAND:
                raise TimeoutError("timeout error with getting current command")
            sleep(0.5)
        return index

class ArmController(DobotCommand):
    """ アーム関連のコマンド群 """
    def __init__(self, dobot:Dobot):
        super().__init__(dobot)
        self.movement = MovementController(dobot)
        self.move_to  = self.movement.exec

    @setting_method
    def set_home_params(self, coord:Mapping[str,SupportsFloat], *, imm:bool = False):
        """ ホームポジションの設定 """
        coord = convert_coord(coord, relative=False)
        param = HomeParams()
        param = coord.infiltrate(param)
        return self.dobot.queue.send(API.SetHOMEParams, args=(byref(param),), imm = imm)

    # 情報取得
    def get_pose(self) -> Tuple[CartCoord, JointCoord]:
        pose = Pose()
        self.dobot.send_command(API.GetPose, args=(byref(pose),))
        return (
            CartCoord(pose.x, pose.y, pose.z, pose.rHead, False),
            JointCoord(pose.joint1Angle, pose.joint2Angle,
                       pose.joint3Angle, pose.joint4Angle, False) )

    def get_pose_in_cartesian(self) -> CartCoord:
        """ 現在のアームの手首部分の座標 """
        return self.get_pose()[0]
    def get_pose_in_joint(self) -> JointCoord:
        """ 現在の各関節の角度 """
        return self.get_pose()[1]

    # Dobot 動作
    def reset_home(self, *, imm:bool = False):
        """ ホームポジションリセット """
        cmd = HOMECmd()
        cmd.temp = 0
        return self.dobot.queue.send(API.SetHOMECmd, args=(byref(cmd),), imm = imm)
    def wait(self, ms:SupportsInt, *, imm:bool = False):
        """ 待機命令 """
        cmd = WAITCmd()
        cmd.waitTime = int(ms)
        return self.dobot.queue.send(API.SetWAITCmd, args=(byref(cmd),), imm = imm)

    def open_gripper(self, *, imm:bool = False):
        """ グリッパーを開く """
        self.dobot.queue.send(API.SetEndEffectorGripper, args=(1,0), imm = imm)
        return self.wait(Dobot.INTERVAL_GRIP)
    def close_gripper(self, *, imm:bool = False):
        """ グリッパーを閉じる """
        self.dobot.queue.send(API.SetEndEffectorGripper, args=(1,1), imm = imm)
        return self.wait(Dobot.INTERVAL_GRIP)
    def stop_pump(self, *, imm:bool = False):
        """ ポンプの停止 """
        return self.dobot.queue.send(API.SetEndEffectorSuctionCup, args=(1,0), imm = imm)

from ptp  import (MovementController)
#from gpio import (IOController)

# エラー
class DobotError(Exception):
    """ Dobot から送出されるエラー """
    pass
class DobotTimeout(DobotError, TimeoutError):
    """ Dobot から送信されるタイムアウトエラー

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
