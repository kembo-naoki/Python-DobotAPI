from abc    import (ABCMeta, abstractmethod)
from enum   import (Enum, auto)
from time   import sleep
from os     import path
from ctypes import (cdll, create_string_buffer, Structure, byref,
                    c_uint64, c_uint32, c_float, c_byte)

_cur_dir = path.dirname( path.abspath( __file__ ) )

class Coordinate(metaclass = ABCMeta):
    @abstractmethod
    def validate(self):
        pass
    
    #@abstractmethod
    def __add__(self, other): pass
    #@abstractmethod
    def __sub__(self, other): pass

    class OutOfRange(Exception):
        pass

class JointCoord(Coordinate):
    def __init__(self, j1, j2, j3, j4, check=True):
        self.J1 = j1
        self.J2 = j2
        self.J3 = j3
        self.J4 = j4
        if check and not self.validate():
            raise Coordinate.OutOfRange()
    
    def validate(self):
        """ 可動域の外ならFalseにする予定 """
        return True

class CartesianCoord(Coordinate):
    def __init__(self, x, y, z, r, check=True):
        self.X = x
        self.Y = y
        self.Z = z
        self.R = r
        if check and not self.validate():
            raise Coordinate.OutOfRange()
    
    def validate(self):
        """ 可動域の外ならFalseにする予定 """
        return True

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

    def send_cmd(self, cmd, *args):
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
    def queue_cmd(self, cmd, *args):
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
        self.send_cmd(cmd, *args, 1, byref(queued_cmd_index))
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
        self.send_cmd(self.api.SetQueuedCmdClear)
    def queued_cmd_start(self):
        """ キューに積まれたコマンドの実行を開始 """
        self.send_cmd(self.api.SetQueuedCmdStartExec)
    def set_home_params(self, x, y, z, r):
        """
        ホームポジションの設定

        Parameters
        ----------
        x : float
            肩の関節の中心を原点とし、前方を正とする座標軸のエンドエフェクタのモーター付け根部分の位置。単位はmm。
        y : float
            アームから見て左側を正とする座標軸。
        z : float
            上方を正とする座標軸。
        r : float
            上から見て反時計回りを正とするエンドエフェクタの角度。単位は度数法。
        """
        param = HomeParams()
        param.x = x
        param.y = y
        param.z = z
        param.r = r
        return self.queue_cmd(self.api.SetHOMEParams, byref(param))
    def set_ptp_joint_prms(self, vel, acc):
        """
        PTP コマンドのモーター毎の速度加速度の設定

        Parameters
        ----------
        vel : list of float
            J1, J2, J3, J4 の各モーターの最高速度。単位はmm/s(0-500)
        acc : list of float
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
        return self.queue_cmd(self.api.SetPTPJointParams, byref(param))
    def set_ptp_common_prms(self, vel, acc):
        """
        PTP コマンド使用時の速度倍率設定

        Parameters
        ----------
        vel : float
            速度の倍率。(0%-100%)
        acc : float
            加速度の倍率。(0%-100%)
        """
        param = PTPCommonParams()
        param.velocityRatio = vel
        param.accelerationRatio = acc
        return self.queue_cmd(self.api.SetPTPCommonParams, byref(param))

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
            self.send_cmd(self.api.GetQueuedCmdCurrentIndex, byref(queued_cmd_index))
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
            "x"(mm), "y"(mm), "z"(mm), "r"(degree)
        """
        pose = Pose()
        self.send_cmd(self.api.GetPose, byref(pose))
        return { "x": pose.x, "y": pose.y, "z": pose.z, "r": pose.rHead }

    # Dobot 動作
    def reset_home(self):
        """ ホームポジションリセット """
        cmd = HOMECmd()
        cmd.temp = 0
        return self.queue_cmd(self.api.SetHOMECmd, byref(cmd))

    class _PTP_MODE(Enum):
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
        REGARDLESS  = auto()
        RECTILINEAR = auto()
        JUMP = auto()
    def ptp_xyz(self, x, y, z, r, mode=DobotAPI.RouteMode.REGARDLESS, inc=False):
        """
        直交座標系で動かす。

        Parameters
        ----------
        x : float
            肩の関節の中心を原点とし、前方を正とする座標軸のエンドエフェクタのモーター付け根部分の位置。単位はmm。
        y : float
            アームから見て左側を正とする座標軸。
        z : float
            上方を正とする座標軸。
        r : float
            上から見て反時計回りを正とするエンドエフェクタの角度。単位は度数法。
        mode : DobotAPI.RouteMode
        inc : bool
            True の時与えられた座標を現在位置からの相対座標として計算する。
    
        Returns
        -------
        cmd_index : int
        """
        if mode is DobotAPI.RouteMode.REGARDLESS:
            if inc : mode = DobotAPI._PTP_MODE.MOVJ_XYZ_INC
            else   : mode = DobotAPI._PTP_MODE.MOVJ_XYZ
        elif mode is DobotAPI.RouteMode.RECTILINEAR:
            if inc : mode = DobotAPI._PTP_MODE.MOVL_INC
            else   : mode = DobotAPI._PTP_MODE.MOVL_XYZ
        elif mode is DobotAPI.RouteMode.JUMP:
            if inc : mode = DobotAPI._PTP_MODE.JUMP_MOVL_XYZ
            else   : mode = DobotAPI._PTP_MODE.JUMP_XYZ
        else : raise TypeError("mode is invalid("+str(mode)+")")
        pos = {"x": x, "y": y, "z": z, "r": r}
        return self._send_ptp_cmd(pos, mode)
    def ptp_joint(self, j1, j2, j3, j4, mode=DobotAPI.RouteMode.REGARDLESS, inc=False):
        """
        サーボモーター単位で動かす。

        Parameters
        ----------
        j1 : float
            アーム根本、上から見て反時計回りを正とする角度。単位は度数法。
        j2 : float
            肩の関節、右から見て反時計回りを正とする角度。
        j3 : float
            肘の関節、右から見て反時計回りを正とする角度。
        j4 : float
            上から見て反時計回りを正とするエンドエフェクタの角度。
        mode : DobotAPI.RouteMode
        inc : bool
            True の時与えられた座標を現在位置からの相対座標として計算する。
    
        Returns
        -------
        cmd_index : int
        """
        if mode is DobotAPI.RouteMode.REGARDLESS:
            if inc : mode = DobotAPI._PTP_MODE.MOVJ_INC
            else   : mode = DobotAPI._PTP_MODE.MOVJ_ANGLE
        elif mode is DobotAPI.RouteMode.RECTILINEAR:
            if inc : raise ValueError("This mode is not supported.(ptp_joint, mode:"+str(mode)+", inc:True)")
            else   : mode = DobotAPI._PTP_MODE.MOVL_ANGLE
        elif mode is DobotAPI.RouteMode.JUMP:
            if inc : raise ValueError("This mode is not supported.(ptp_joint, mode:"+str(mode)+", inc:True)")
            else   : mode = DobotAPI._PTP_MODE.JUMP_ANGLE
        else : raise TypeError("mode is invalid("+str(mode)+")")
        pos = {"x": j1, "y": j2, "z": j3, "r": j4}
        return self._send_ptp_cmd(pos, mode)
    def _send_ptp_cmd(self, pos, mode):
        """
        Parameter
        ---------
        pos : dict of {str: float}
        mode : DobotAPI._PTP_MODE
        """
        cmd = PTPCmd()
        cmd.ptpMode = mode.value
        cmd.x = pos["x"]
        cmd.y = pos["y"]
        cmd.z = pos["z"]
        r = pos["r"]
        if   r < -180 : r += 360
        elif r >  180 : r -= 360
        cmd.rHead = r
        return self.queue_cmd(self.api.SetPTPCmd, byref(cmd))

    def open_gripper(self):
        """ グリッパーを開く """
        self.queue_cmd(self.api.SetEndEffectorGripper, 1, 0)
        return self.wait_cmd(DobotAPI.INTERVAL_GRIP)
    def close_gripper(self):
        """ グリッパーを閉じる """
        self.queue_cmd(self.api.SetEndEffectorGripper, 1, 1)
        return self.wait_cmd(DobotAPI.INTERVAL_GRIP)
    def stop_pump(self):
        """ ポンプの停止 """
        return self.queue_cmd(self.api.SetEndEffectorSuctionCup, 1, 0)

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
        return self.queue_cmd(self.api.SetWAITCmd, byref(cmd))
    def force_stop(self):
        """ 強制停止 """
        self.stop_flg = True
        self.send_cmd(self.api.SetQueuedCmdForceStopExec)
    def restart(self):
        """ 強制停止後の再開命令 """
        self.stop_flg = False
    
    # 外部IO
    class IOMode(Enum):
        INVALID = 0
        LEVEL_OUTPUT = 1
        PWM_OUTPUT = 2
        LEVEL_INPUT = 3
        AD_INPUT = 4
        # PULLUP_INPUT = 5
        # PULLDOWN_INPUT = 6
    class Pin(object):
        def __init__(self, address, volt, permission=[]):
            """
            Parameters
            ----------
            address : int
            volt : float
            permission : list of IOMode
            """
            self.address = address
            self.volt = volt
            self.permission = set([DobotAPI.IOMode.INVALID]+list(permission))
            self.mode = None
        def set_mode(self, mode):
            self.mode = mode
    class Interface(object):
        def __init__(self, *pins):
            """
            Parameters
            ----------
            label : str
            pins : (int, str, DobotAPI._Interface._Pin)
            """
            self.pins = {}
            for pin in pins:
                self.pins[pin[0]] = pin[2]
                self.pins[pin[1]] = pin[2]
    _LvOut = IOMode.LEVEL_OUTPUT
    _LvIn  = IOMode.LEVEL_INPUT
    _PWM = IOMode.PWM_OUTPUT
    _ADC = IOMode.AD_INPUT
    _heat_12v = Pin(3, 12, [_LvOut])
    Interfaces = {
        "base": {
            "UART": Interface( (3, "E2",       Pin(18, 3.3, [_LvOut])),
                                (7, "STOP_KEY", Pin(20, 3.3, [_LvIn ])),
                                (8, "E1",       Pin(19, 3.3, [_LvIn ])) ),

            "GP1" : Interface( (1, "REV", Pin(10, 5.0, [_LvOut      ])),
                                (2, "PWM", Pin(11, 3.3, [_LvOut, _PWM])),
                                (3, "ADC", Pin(12, 3.3, [_LvIn       ])) ),
            "GP2" : Interface( (1, "REV", Pin(13, 5.0, [_LvOut])),
                                (2, "PWM", Pin(14, 3.3, [_LvOut, _LvIn, _PWM])),
                                (3, "ADC", Pin(15, 3.3, [_LvOut, _LvIn, _ADC])) ),
            "SW1" : Interface( (1, "VALVE", Pin(16, 12, [_LvOut]))),
            "SW2" : Interface( (1, "PUMP" , Pin(16, 12, [_LvOut])))
        },
        "arm": {
            "GP3": Interface( (2, "PWM", Pin(8, 3.3, [_LvOut, _PWM       ])),
                               (3, "ADC", Pin(9, 3.3, [_LvOut, _LvIn, _ADC])) ),
            "GP4": Interface( (2, "PWM", Pin(6, 3.3, [_LvOut, _PWM])),
                               (3, "ADC", Pin(7, 3.3, [_LvIn       ])) ),
            "GP5": Interface( (2, "PWM", Pin(4, 3.3, [_LvOut, _PWM])),
                               (3, "ADC", Pin(5, 3.3, [_LvIn       ])) ),
            "SW3": Interface( (2, "HEAT_12V", _heat_12v), (3, "HEAT_12V", _heat_12v) ),

            "SW4":    Interface( (1, "FAN_12V", Pin(2, 12, [_LvOut])) ),
            "ANALOG": Interface( (1, "Temp", Pin(1, 3.3, [_LvOut, _LvIn, _ADC])) )
        }
    }
    def config_io(self, pin, mode):
        """
        特定の I/O ポートのモード設定

        Parameters
        ----------
        pin : DobotAPI.Pin
        mode : DobotAPI.IOMode
        """
        cmd = TagIOMultiplexing()
        cmd.address = pin.address
        cmd.mode = mode.value
        result = self.queue_cmd(self.api.SetIOMultiplexing, byref(cmd))
        pin.set_mode(mode)
        return result
    # Level I/O
    class Level(Enum):
        LOW = 0
        HIGH = 1
    def level_out(self, pin, level):
        """
        特定の I/O ポートの出力設定

        Parameters
        ----------
        pin : DobotAPI.Pin
        mode : DobotAPI.Level
        """
        cmd = TagIODO()
        cmd.address = pin.address
        cmd.level   = level.value
        return self.queue_cmd(self.api.SetIODO, byref(cmd))


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
