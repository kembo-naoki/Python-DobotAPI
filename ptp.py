from enum   import Enum
from ctypes import (Structure, byref, c_float, c_byte)

from base import (CommandModule, API)
from coordinate import (convert_coord, Coordinate,
    CartCoord, CartVector, JointCoord, JointVector)

class MoveController(CommandModule):
    class PTPMode(Enum):
        JUMP_XYZ   = 0
        MOVJ_XYZ   = 1
        MOVL_XYZ   = 2
        JUMP_ANGLE = 3
        MOVJ_ANGLE = 4
        MOVL_ANGLE = 5
        MOVJ_INC   = 6
        MOVL_INC   = 7
        MOVJ_XYZ_INC = 8
        JUMP_MOVL_XYZ= 9
    class RouteMode(Enum):
        REGARDLESS = 0
        LINEAR     = 1
        JUMP       = 2
    
    AXES_LIST = {
        "Cartesian": ("x", "y", "z", "r"),
        "Joint": ("j1", "j2", "j3", "j4")
    }
    MODE_LIST = {
        RouteMode.REGARDLESS: {
            CartCoord:  PTPMode.MOVJ_XYZ,
            CartVector: PTPMode.MOVJ_XYZ_INC,
            JointCoord: PTPMode.MOVJ_ANGLE,
            JointVector: PTPMode.MOVJ_INC
        },
        RouteMode.LINEAR: {
            CartCoord:  PTPMode.MOVL_XYZ,
            CartVector: PTPMode.MOVL_INC,
            JointCoord: PTPMode.MOVL_ANGLE,
            JointVector: "You can't use RouteMode.LINEAR with `JointVector`."
        },
        RouteMode.JUMP: {
            CartCoord:  PTPMode.JUMP_XYZ,
            CartVector: PTPMode.JUMP_MOVL_XYZ,
            JointCoord: PTPMode.JUMP_ANGLE,
            JointVector: "You can't use RouteMode.JUMP with `JointVector`."
        }
    }


    def __init__(self, dobot):
        self.dobot = dobot
        self.check_list = {
            "joint_prms": False,
            "common_ratio": False }
        self.requirements = ("joint_prms", "common_ratio")

    def set_joint_prms(self, vel, acc, *, imm=False):
        """
        MoveController コマンドのモーター毎の速度加速度の設定

        Parameters
        ----------
        vel: list of float
            J1, J2, J3, J4 の各モーターの最高速度。単位はmm/s(0-500)
        acc: list of float
            J1, J2, J3, J4 の各モーターの加速度。単位はmm/s(0-500)
        imm: bool
            即座に実行する
        """
        param = PTPJointParams()
        param.joint1Velocity = vel[0]
        param.joint2Velocity = vel[1]
        param.joint3Velocity = vel[2]
        param.joint4Velocity = vel[3]
        param.joint1Acceleration = acc[0]
        param.joint2Acceleration = acc[1]
        param.joint3Acceleration = acc[2]
        param.joint4Acceleration = acc[3]
        self.dobot.queue.send(API.SetMoveControllerJointParams, byref(param), imm=imm)
        self.check_list["joint_prms"] = True

    def set_common_ratio(self, vel, acc, *, imm=False):
        """
        MoveController コマンド使用時の速度倍率設定

        Parameters
        ----------
        vel: float
            速度の倍率。(0%-100%)
        acc: float
            加速度の倍率。(0%-100%)
        imm: bool
            即座に実行する
        """
        param = PTPCommonParams()
        param.velocityRatio = vel
        param.accelerationRatio = acc
        self.dobot.queue.send(API.SetMoveControllerCommonParams, byref(param), imm=imm)
        self.check_list["common_ratio"] = True

    def exec(self, point, relative=None, mode=RouteMode.REGARDLESS, *, imm=False):
        """
        アームを特定の座標まで任意の経路で動かす。

        Parameters
        ----------
        point: (Coordinate, dict)
            目的地を表す。
            dobot.coordinate モジュールで定義されているクラスのインスタンス、
            もしくはx,y,z,rまたはj1,j2,j3,j4による座標の指定。
        relative: bool
            True ならば point を相対座標として動かす。省略した場合 point の型に従い、
            point が dict の場合は False になります。
        mode: MoveController
            REGARDLESS = 各関節の動きを優先して最適な経路で向かう
            LINEAR = 直線的な経路で向かう
            JUMP = ジャンプするように一度持ち上げて下ろす経路
        imm: bool
            True のとき即座に実行する。

        Returns
        -------
        cmd_index: int
        """
        if relative is None:
            if isinstance(point, Coordinate):
                relative = point.is_relative
            else:
                relative = False
        point = convert_coord(point, relative)

        cmd = PTPCmd()
        mode = self.MODE_LIST[mode][type(point)]
        if not isinstance(mode, self.PTPMode):
            raise ValueError(mode["message"])
        cmd.ptpMode = mode.value
        cmd = point.infiltrate(cmd, "x", "y", "z", "rHead")
        return self.dobot.queue.send(API.SetPTPCmd, byref(cmd), imm=imm)

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
