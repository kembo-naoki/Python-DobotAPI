from enum   import (Enum, auto)
from ctypes import (Structure, byref, c_float, c_byte)

from main import (CommandModule, API)
from coordinate import *

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
        REGARDLESS = auto()
        LINEAR = auto()
        JUMP = auto()
    
    AXES_LIST = {
        "Cartesian": ("x", "y", "z", "r"),
        "Joint": ("j1", "j2", "j3", "j4")
    }

    MODE_LIST = {
        RouteMode.REGARDLESS: {
            CartesianCoordinate: {
                "absolute": PTPMode.MOVJ_XYZ,
                "relative": PTPMode.MOVJ_XYZ_INC
            },
            JointCoordinate: {
                "absolute": PTPMode.MOVJ_ANGLE,
                "relative": PTPMode.MOVJ_INC
            }
        },
        RouteMode.LINEAR: {
            CartesianCoordinate: {
                "absolute": PTPMode.MOVL_XYZ,
                "relative": PTPMode.MOVL_INC
            },
            JointCoordinate: {
                "absolute": PTPMode.MOVL_ANGLE,
                "relative": {"message": "You can't use `relative` mode with `JointCoordinate`."}
            }
        },
        RouteMode.JUMP: {
            CartesianCoordinate: {
                "absolute": PTPMode.JUMP_XYZ,
                "relative": PTPMode.JUMP_MOVL_XYZ
            },
            JointCoordinate: {
                "absolute": PTPMode.JUMP_ANGLE,
                "relative": {"message": "You can't use `relative` mode with `JointCoordinate`."}
            }
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
        param = MoveControllerJointParams()
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
        param = MoveControllerCommonParams()
        param.velocityRatio = vel
        param.accelerationRatio = acc
        self.dobot.queue.send(API.SetMoveControllerCommonParams, byref(param), imm=imm)
        self.check_list["common_ratio"] = True

    def _exec(self, coord, relative, route_mode, *, imm=False):
        """
        アームを特定の座標まで動かす。

        Parameters
        ----------
        coord: Coordinate
        route_mode: MoveController.Mode
        relative: bool
        imm: bool
            True のとき即座に実行する

        Returns
        -------
        cmd_index: int
        """
        cmd = PTPCmd()
        pos = "relative" if relative else "absolute"
        mode = MoveController.MODE_LIST[route_mode][type(coord)][pos]
        if not isinstance(mode, MoveController.PTPMode):
            raise ValueError(mode["message"])
        cmd.ptpMode = mode.value
        cmd = coord.infiltrate(cmd, "x", "y", "z", "rHead")
        return self.dobot.queue.send(API.SetMoveControllerCmd, byref(cmd), imm=imm)

    def move_to(self, point, relative=None, *,mode=RouteMode.REGARDLESS , imm=False):
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
        if isinstance(point, Coordinate):
            if relative is None:
                relative = isinstance(point, RelativeCoordinate)
        elif isinstance(point, dict):
            if relative is None:
                relative = False
            if all(axis in point for axis in AXES_LIST["Cartesian"]):
                if relative:
                    point = CartesianRelativeCoordinate(**point)
                else:
                    point = CartesianAbsoluteCoordinate(**point)
            elif all(axis in point for axis in AXES_LIST["Joint"]):
                if relative:
                    point = JointRelativeCoordinate(**point)
                else:
                    point = JointAbsoluteCoordinate(**point)
            else:
                raise ValueError("`point` must have keys (x,y,z,r) or (j1,j2,j3,j4).")
        else:
            raise TypeError("`point` must be dict or dobot.coordinate.Coordinate.")

        return self._exec(point, relative, mode, imm=imm)

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
