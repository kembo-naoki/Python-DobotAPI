from enum   import Enum
from ctypes import (Structure, byref, c_float, c_byte)
from typing import (List, Dict, Optional, Union, SupportsFloat)

from main import (API, setting_method, DobotCommand, Dobot)
from coordinate import (convert_coord, Coordinate,
    CartesianCoordinateSystem, JointCoordinateSystem,
    CartCoord, CartVector, JointCoord, JointVector)

class MovementController(DobotCommand):
    """ アームの特定座標から特定座標への基本的な動きを司るクラス """
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
        JUMP_MOVL_XYZ= 9
    class RouteMode(Enum):
        REGARDLESS = 0
        LINEAR     = 1
        JUMP       = 2
    class CoordinateSystem(Enum):
        Cartesian = 0
        Joint = 1
    
    MODE_LIST = [
        [
            {
                RouteMode.REGARDLESS: _PTPMode.MOVJ_ANGLE,
                RouteMode.LINEAR:     _PTPMode.MOVL_ANGLE,
                RouteMode.JUMP:       _PTPMode.JUMP_ANGLE
            }, {
                RouteMode.REGARDLESS: _PTPMode.MOVJ_INC,
                RouteMode.LINEAR:
                    ValueError("You can't use `RouteMode.LINEAR` "
                               "and increment mode with `JointCoordinate`."),
                RouteMode.JUMP:
                    ValueError("You can't use `RouteMode.JUMP` "
                               "and increment mode with `JointCoordinate`.")
            }
        ], [
            {
                RouteMode.REGARDLESS: _PTPMode.MOVJ_XYZ,
                RouteMode.LINEAR:     _PTPMode.MOVL_XYZ,
                RouteMode.JUMP:       _PTPMode.JUMP_XYZ
            }, {
                RouteMode.REGARDLESS: _PTPMode.MOVJ_XYZ_INC,
                RouteMode.LINEAR:     _PTPMode.MOVL_INC,
                RouteMode.JUMP:       _PTPMode.JUMP_MOVL_XYZ
            }
        ]
    ]

    def __init__(self, dobot:'Dobot'):
        super().__init__(dobot)
        self.set_mode()

    @setting_method
    def set_joint_prms(self, vel:List[SupportsFloat],
                             acc:List[SupportsFloat], *, imm:bool = False):
        """ モーター毎の速度加速度の設定

        vel
            J1, J2, J3, J4 の各モーターの最高速度。単位はmm/s(0-500)
        acc
            J1, J2, J3, J4 の各モーターの加速度。単位はmm/s(0-500)
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
        self.dobot.queue.send(API.SetMoveControllerJointParams, (byref(param),), imm = imm)

    @setting_method
    def set_common_ratio(self, vel:SupportsFloat,
                               acc:SupportsFloat, *, imm:bool = False):
        """ 速度と加速度の倍率設定

        vel
            速度の倍率。(0%-100%)
        acc
            加速度の倍率。(0%-100%)
        """
        param = PTPCommonParams()
        param.velocityRatio = vel
        param.accelerationRatio = acc
        self.dobot.queue.send(API.SetMoveControllerCommonParams, (byref(param),), imm = imm)
    
    @setting_method
    def set_mode(self, cartesian:bool = True, increment:bool = False,
                 route:RouteMode = RouteMode.REGARDLESS):
        """ Dobot の移動先の指定方法や経路を設定します。 """
        self._mode = (cartesian, increment, route) # type: Tuple[bool, bool, MovementController.RouteMode]
    
    @property
    def mode(self) -> Dict:
        return {
            'cartesian': self._mode[0],
            'increment': self._mode[1],
            'route': self._mode[2]
        }
        
    def exec(self, dest:Union[Coordinate,Dict[str,SupportsFloat]], *, imm:bool = False):
        """ アームを特定の座標まで経路を選択して動かす。 """
        dest = convert_coord(dest, self._mode[1])

        just_cart  = self._mode[0] and isinstance(dest, CartesianCoordinateSystem)
        just_joint = (not self._mode[0]) and isinstance(dest, JointCoordinateSystem)
        if not(just_cart or just_joint):
            raise TypeError("not match coordinate system.")

        mode = self.MODE_LIST[self._mode[0]][self._mode[1]][self._mode[2]]
        if not isinstance(mode, Exception):
            raise mode

        cmd = PTPCmd()
        cmd.ptpMode = mode.value
        cmd = dest.infiltrate(cmd, {"r": "rHead", "j1": "x", "j2": "y", "j3": "z", "j4": "rHead"})
        return self.dobot.queue.send(API.SetPTPCmd, (byref(cmd),), imm = imm)

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
