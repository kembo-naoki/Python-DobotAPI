from enum   import Enum
from ctypes import (Structure, byref, c_float, c_byte)
from typing import (List, Dict, Optional, Union, SupportsFloat)

from base import (CommandModule, API)
from coordinate import (convert_coord, Coordinate,
    CartCoord, CartVector, JointCoord, JointVector)

class MoveController(CommandModule):
    """ アームの特定座標から特定座標への基本的な動きを司るクラス """
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


    def __init__(self, dobot:'Dobot'):
        self.dobot = dobot
        self.check_list = {
            "joint_prms": False,
            "common_ratio": False }
        self.requirements = ("joint_prms", "common_ratio")

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
        self.dobot.queue.send(API.SetMoveControllerJointParams, byref(param), imm = imm)
        self.check_list["joint_prms"] = True

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
        self.dobot.queue.send(API.SetMoveControllerCommonParams, byref(param), imm = imm)
        self.check_list["common_ratio"] = True

    def exec(self, dest:Union[Coordinate,Dict[str,SupportsFloat]], relative:Optional[bool]=None,
                   mode:'MoveController.RouteMode'=RouteMode.REGARDLESS, *, imm:bool = False):
        """ アームを特定の座標まで経路を選択して動かす。 """
        if relative is None:
            if isinstance(dest, Coordinate):
                relative = dest.is_relative
            else:
                relative = False
        dest = convert_coord(dest, relative)

        cmd = PTPCmd()
        mode = self.MODE_LIST[mode][type(dest)]
        if not isinstance(mode, self.PTPMode):
            raise ValueError(mode["message"])
        cmd.ptpMode = mode.value
        cmd = dest.infiltrate(cmd, {"r": "rHead", "j1": "x", "j2": "y", "j3": "z", "j4": "rHead"})
        return self.dobot.queue.send(API.SetPTPCmd, byref(cmd), imm = imm)

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
