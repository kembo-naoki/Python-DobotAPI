from ctypes import (Structure, byref, c_float, c_uint32)
from typing import (SupportsFloat, SupportsInt, Tuple, Mapping)

from .base import (API, DobotClient, setting_method, DobotCommand,
                   DobotConnectionError, DobotTimeout)
from .ptp import (MovementController)
from .queue import QueueController
# from .gpio import (IOController)
from .coordinate import (convert_coord, CartCoord, JointCoord)


class Dobot(DobotClient):
    """ Dobot 本体を表すクラス """
    def __init__(self, logger=None):
        """ インスタンスを作成した時点ではまだ接続はしません。 """
        self.is_connected = False

        self.queue = QueueController(self)
        self.arm = ArmController(self)
        # self.io = IOController(self)

        if logger is not None:
            self.logger = logger

    # コマンドそのものに関するメソッド
    def send_command(self, cmd: API._FuncPtr, args: tuple = tuple()):
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


class ArmController(DobotCommand):
    """ アーム関連のコマンド群 """

    INTERVAL_CMD = 0.005
    INTERVAL_GRIP = 1000

    def __init__(self, dobot: Dobot):
        super().__init__(dobot)
        self.movement = MovementController(dobot)
        self.move_to = self.movement.exec

    @setting_method
    def set_home_params(self, coord: Mapping[str, SupportsFloat], *,
                        imm: bool = False):
        """ ホームポジションの設定 """
        coord = convert_coord(coord, relative=False)
        param = HomeParams()
        param = coord.infiltrate(param)
        return self.dobot.queue.send(
            API.SetHOMEParams, args=(byref(param),), imm=imm)

    # 情報取得
    def get_pose(self) -> Tuple[CartCoord, JointCoord]:
        pose = Pose()
        self.dobot.send_command(API.GetPose, args=(byref(pose),))
        return (
            CartCoord(pose.x, pose.y, pose.z, pose.rHead, False),
            JointCoord(pose.joint1Angle, pose.joint2Angle,
                       pose.joint3Angle, pose.joint4Angle, False))

    def get_pose_in_cartesian(self) -> CartCoord:
        """ 現在のアームの手首部分の座標 """
        return self.get_pose()[0]

    def get_pose_in_joint(self) -> JointCoord:
        """ 現在の各関節の角度 """
        return self.get_pose()[1]

    # Dobot 動作
    def reset_home(self, *, imm: bool = False):
        """ ホームポジションリセット """
        cmd = HOMECmd()
        cmd.temp = 0
        return self.dobot.queue.send(
            API.SetHOMECmd, args=(byref(cmd),), imm=imm)

    def wait(self, ms: SupportsInt, *, imm: bool = False):
        """ 待機命令 """
        cmd = WAITCmd()
        cmd.waitTime = int(ms)
        return self.dobot.queue.send(
            API.SetWAITCmd, args=(byref(cmd),), imm=imm)

    def open_gripper(self, *, imm: bool = False):
        """ グリッパーを開く """
        self.dobot.queue.send(API.SetEndEffectorGripper, args=(1, 0), imm=imm)
        return self.wait(Dobot.INTERVAL_GRIP)

    def close_gripper(self, *, imm: bool = False):
        """ グリッパーを閉じる """
        self.dobot.queue.send(API.SetEndEffectorGripper, args=(1, 1), imm=imm)
        return self.wait(Dobot.INTERVAL_GRIP)

    def stop_pump(self, *, imm: bool = False):
        """ ポンプの停止 """
        return self.dobot.queue.send(
            API.SetEndEffectorSuctionCup, args=(1, 0), imm=imm)


# コマンド用の構造体群


class HomeParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float)]


class HOMECmd(Structure):
    _pack_ = 1
    _fields_ = [("temp", c_float)]


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
        ("joint4Angle", c_float)]


class WAITCmd(Structure):
    _pack_ = 1
    _fields_ = [("waitTime", c_uint32)]
