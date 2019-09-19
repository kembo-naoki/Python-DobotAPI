from enum import Enum
from ctypes import (Structure, byref, c_float, c_byte)

from .base import (DobotServer, AbstractDobotServer, AbstractDobotService)
from .queue import (QueueServer, AbstractDobotQueueService)
from .coordinate import (CartCoord, JointCoord)


class PointToPointMovementServer(AbstractDobotServer):
    """Dobot のアームを2点間移動させるための機能群"""
    DEF_JOINT_PRMS = {
        "vel": JointCoord(320, 320, 320, 320),
        "acc": JointCoord(160, 160, 160, 320)
    }

    def __init__(self, queue: QueueServer):
        self.queue = queue

        self._set_joint_prms = SettingJointParamService(self)
        self.joint_prms = {"vel": None, "acc": None}

    def set_default_setting(self, mode: str = "Cart") -> None:
        """動作用の設定を適当な初期値にセット

        初期値はクラス変数 DEF_JOINT_PRMS, DEF_CART_PRMS, DEF_COM_RATIO
        に格納されている。
        Args:
            mode: "Cart"または"Joint"
        """
        mode = mode.lower()
        if mode == "cart":
            pass
        if mode == "joint":
            self.set_joint_prms(**self.DEF_JOINT_PRMS)

    def set_joint_prms(self, vel: JointCoord, acc: JointCoord,
                       *, imm: bool = False) -> int:
        result = self._set_joint_prms(vel, acc, imm=imm)
        self.joint_prms["vel"] = result["vel"]
        self.joint_prms["acc"] = result["acc"]
        return result["cmd_idx"]


class _PTPMode(Enum):
    JUMP_XYZ = 0
    MOVJ_XYZ = 1
    MOVL_XYZ = 2
    JUMP_ANGLE = 3
    MOVJ_ANGLE = 4
    MOVL_ANGLE = 5
    MOVJ_INC = 6
    MOVL_INC = 7
    MOVJ_XYZ_INC = 8
    JUMP_MOVL_XYZ = 9


class RouteMode(Enum):
    REGARDLESS = 0
    LINEAR = 1
    JUMP = 2


class CoordSystem(Enum):
    Cart = 0
    Joint = 1


MODE_LIST = {
    CoordSystem.Cart: {
        RouteMode.REGARDLESS: [_PTPMode.MOVJ_XYZ, _PTPMode.MOVJ_XYZ_INC ],
        RouteMode.LINEAR:     [_PTPMode.MOVL_XYZ, _PTPMode.MOVL_INC     ],
        RouteMode.JUMP:       [_PTPMode.JUMP_XYZ, _PTPMode.JUMP_MOVL_XYZ]
    },
    CoordSystem.Joint: {
        RouteMode.REGARDLESS: [_PTPMode.MOVJ_ANGLE, _PTPMode.MOVJ_ANGLE],
        RouteMode.LINEAR: [
            _PTPMode.MOVL_ANGLE,
            ValueError("You can't use `RouteMode.LINEAR` "
                       "and increment mode with `JointCoord`.")
        ],
        RouteMode.JUMP: [
            _PTPMode.JUMP_ANGLE,
            ValueError("You can't use `RouteMode.JUMP` "
                       "and increment mode with `JointCoord`.")
        ]
    }
}


class _AbstractPTPService(AbstractDobotService):
    """PTP 関連のコマンドの内、Queue を使用しないもの"""
    def __init__(self, server: PointToPointMovementServer):
        self.server = server


class _AbstractPTPServiceQ(_AbstractPTPService, AbstractDobotQueueService):
    """PTP 関連のコマンドの内、Queue を使用するもの"""
    pass


class C_JointParams(Structure):
    _fields_ = [
        ("joint1Velocity", c_float),
        ("joint2Velocity", c_float),
        ("joint3Velocity", c_float),
        ("joint4Velocity", c_float),
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)]


class SettingJointParamService(_AbstractPTPServiceQ):
    """アーム移動コマンドの Joint 座標系用の動作設定をするコマンド"""
    COMMAND = DobotServer.Lib.SetMoveControllerJointParams

    def __call__(self, vel: JointCoord, acc: JointCoord,
                 *, imm: bool = False) -> dict:
        """アーム移動コマンドの Joint 座標系用の動作設定

        Args:
            vel: 各関節の最大速度(°/s)(0-320)
            acc: 各関節の加速度(°/s^2)(0-?)
            imm: このコマンドをキューに積まず、即時実行したい場合のみ True
        """
        params = C_JointParams(*vel.values(), *acc.values())
        index = self._send_cmd(params)
        return {
            "cmd_idx": index,
            "vel": vel, "acc": acc
        }


"""
    @setting_method
    def set_common_ratio(self, vel: SupportsFloat,
                         acc: SupportsFloat, *, imm: bool = False):

        vel
            速度の倍率。(0%-100%)
        acc
            加速度の倍率。(0%-100%)
        param = PTPCommonParams()
        param.velocityRatio = vel
        param.accelerationRatio = acc
        self.dobot.queue.send(
            API.SetMoveControllerCommonParams, (byref(param),), imm=imm)

    @setting_method
    def set_mode(self, cartesian: bool = True, increment: bool = False,
                 route: RouteMode = RouteMode.REGARDLESS):
        self._mode = (cartesian, increment, route)\
            # type: Tuple[bool, bool, MovementController.RouteMode]

    @property
    def mode(self) -> Dict:
        return {
            'cartesian': self._mode[0],
            'increment': self._mode[1],
            'route': self._mode[2]
        }

    def exec(self, dest: Union[Coordinate, Dict[str, SupportsFloat]], *,
             imm: bool = False):
        dest = convert_coord(dest, self._mode[1])

        just_cart = self._mode[0] and isinstance(
            dest, CartesianCoordinateSystem)
        just_joint = (not self._mode[0]) and isinstance(
            dest, JointCoordinateSystem)
        if not(just_cart or just_joint):
            raise TypeError("not match coordinate system.")

        mode = self.MODE_LIST[self._mode[0]][self._mode[1]][self._mode[2]]
        if not isinstance(mode, Exception):
            raise mode

        cmd = PTPCmd()
        cmd.ptpMode = mode.value
        rabel_replacements = {
            "r": "rHead", "j1": "x", "j2": "y", "j3": "z", "j4": "rHead"}
        cmd = dest.infiltrate(cmd, rabel_replacements)
        return self.dobot.queue.send(API.SetPTPCmd, (byref(cmd),), imm=imm)


class PTPJointParams(Structure):
    _fields_ = [
        ("joint1Velocity", c_float),
        ("joint2Velocity", c_float),
        ("joint3Velocity", c_float),
        ("joint4Velocity", c_float),
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)]


class PTPCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float),
        ("accelerationRatio", c_float)]


class PTPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float)]
"""
