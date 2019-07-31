from ctypes import (Structure, byref, c_byte, c_float, c_ushort)
from enum   import Enum
from typing import (SupportsFloat, List, Mapping)

from base import (CommandModule, API)

class IOController(CommandModule):
    """ Dobot の各種 I/Oインターフェイスをまとめるためのクラス。 """
    class Mode(Enum):
        """ 出力方式 """
        INVALID = 0
        LEVEL_OUTPUT = 1
        PWM_OUTPUT = 2
        LEVEL_INPUT = 3
        AD_INPUT = 4
        # PULLUP_INPUT = 5
        # PULLDOWN_INPUT = 6
    def __init__(self, dobot:'Dobot'):
        self.dobot = dobot
        # 設定用の仮の定数
        LvOut = IOController.Mode.LEVEL_OUTPUT
        LvIn  = IOController.Mode.LEVEL_INPUT
        PWM   = IOController.Mode.PWM_OUTPUT
        ADC   = IOController.Mode.AD_INPUT
        # ベース部分のインターフェイス
        self.uart = UART(dobot, E1=Pin(dobot, 19, 3.3, [LvIn]),
                                E2=Pin(dobot, 18, 3.3, [LvOut]),
                                STOP_KEY=Pin(dobot, 20, 3.3, [LvIn]))
        self.gp1 = GPOfBase(dobot, REV=Pin(dobot, 10, 5.0, [LvOut]),
                                   PWM=Pin(dobot, 11, 3.3, [LvOut, PWM]),
                                   ADC=Pin(dobot, 12, 3.3, [LvIn]))
        self.gp2 = GPOfBase(dobot, REV=Pin(dobot, 13, 5.0, [LvOut]),
                                   PWM=Pin(dobot, 14, 3.3, [LvOut, LvIn, PWM]),
                                   ADC=Pin(dobot, 15, 3.3, [LvOut, LvIn, ADC]))
        self.sw1 = SW1(dobot, VALVE=Pin(dobot, 16, 12, [LvOut]))
        self.sw2 = SW2(dobot, PUMP=Pin(dobot, 16, 12, [LvOut]))
        # アーム部分のインターフェイス
        self.gp3 = GPOfArm(dobot, PWM=Pin(dobot, 8, 3.3, [LvOut, PWM]),
                                  ADC=Pin(dobot, 9, 3.3, [LvOut, LvIn, ADC]))
        self.gp4 = GPOfArm(dobot, PWM=Pin(dobot, 6, 3.3, [LvOut, PWM]),
                                  ADC=Pin(dobot, 7, 3.3, [LvIn]))
        self.gp5 = GPOfArm(dobot, PWM=Pin(dobot, 4, 3.3, [LvOut, PWM]),
                                  ADC=Pin(dobot, 5, 3.3, [LvIn]))
        self.sw3 = SW3(dobot, HEAT_12V=Pin(dobot, 3, 12, [LvOut]))
        self.sw4 = SW4(dobot, FAN_12V=Pin(dobot, 2, 12, [LvOut]))
        self.analog = ANALOG(dobot, Temp=Pin(dobot, 1, 3.3, [LvOut, LvIn, ADC]))

class Interface(CommandModule):
    """ Dobot の各ポートに対応するクラス。 """
    def __init__(self, dobot:'Dobot', **kwargs:Mapping[str,'Pin']):
        self.dobot = dobot
        self.PINS_BY_POS = {}
        for label in kwargs:
            if not label in self.PIN_POS:
                raise TypeError(str(type(self))+" Interface doesn't have `"+label+"` Pin")
            pin = kwargs[label]
            setattr(self, label, pin)
            self.PINS_BY_POS[self.PIN_POS[label]] = pin
        if len(kwargs) < len(self.PIN_POS):
            raise TypeError(str(type(self))+" Interface is missing any pin")
    def __getitem__(self, key:str) -> int:
        return self.PINS_BY_POS[key]

# 様々なタイプのポートを一括定義
class UART(Interface):
    """ WiFi や Bluetooth を接続するための UART ポート """
    PIN_POS = {"E1": 8, "E2": 3, "STOP_KEY": 7}
class SW1(Interface):
    """ 主にエアーポンプを制御するためのポート """
    PIN_POS = {"VALVE": 1}
class SW2(Interface):
    """ 12V出力をコントロールするためのポート """
    PIN_POS = {"PUMP":  1}
class SW3(Interface):
    """ 3Dプリンタ用のホットエンドエフェクタ制御用12V出力ポート """
    def __init__(self, dobot, HEAT_12V:int):
        self.dobot = dobot
        pin = HEAT_12V
        self.HEAT_12V = pin
        self.PINS_BY_POS = {2: pin, 3: pin}
class SW4(Interface):
    """ 3Dプリンタ用のファン制御用12V出力ポート """
    PIN_POS = {"FAN_12V": 1}
class ANALOG(Interface):
    """ 3Dプリンタ用のサーミスタ（温度センサー）からのアナログ入力ポート """
    PIN_POS = {"Temp": 1}
class GPOfBase(Interface):
    """ 土台側の GPIO """
    PIN_POS = {"REV": 1, "PWM": 2, "ADC": 3}
class GPOfArm(Interface):
    """ アーム側の GPIO """
    PIN_POS = {"PWM": 2, "ADC": 3}

class Pin(CommandModule):
    """ 各ポートのピン1つ1つに対応するクラス。 """
    def __init__(self, dobot:'Dobot', address:int, volt:SupportsFloat,
                       usage:List[IOController.Mode] = []):
        self.ADDRESS = address
        self.VOLT = float(volt)
        self.USAGE = set([IOController.Mode.INVALID]+usage)
        self.dobot = dobot
        self.mode = None

    def config(self, mode:IOController.Mode, *, imm:bool = False):
        """ I/O モードの設定 """
        if not mode in self.USAGE:
            raise ValueError("this mode is not supported ("+mode._name_+")")
        cmd = IOMultiplexing()
        cmd.address = self.ADDRESS
        cmd.mode = mode.value
        result = self.dobot.queue.send(API.SetIOMultiplexing, byref(cmd), imm = imm)
        pin.set_mode(mode)
        return result

    def _check_mode(self, mode:IOController.Mode):
        """ ポートに適切なモード設定がなされているかどうかを確認 """
        if self.mode is not mode:
            raise RuntimeError("This pin is not mode `"+mode._name_+"`")

    # Output
    def level_out(self, level:bool, *, imm:bool = False):
        """ このピンの出力 ON / OFF """
        self._check_mode(IOController.Mode.LEVEL_OUTPUT)
        cmd = IODO()
        cmd.address = self.ADDRESS
        cmd.level   = int(level)
        return self.dobot.queue.send(API.SetIODO, byref(cmd), imm = imm)

    def pwm_out(self, freq:SupportsFloat, duty:SupportsFloat, *, imm:bool = False):
        """ このピンの PWM 出力

        freq は 10-1,000,000Hz の PWM の周波数を表します。
        duty は 0-100% のデューティ比を表します。
        """
        self._check_mode(IOController.Mode.PWM_OUTPUT)
        cmd = IOPWM()
        cmd.address = self.ADDRESS
        cmd.frequency = freq
        cmd.dutyCycle = duty
        return self.dobot.queue.send(API.SetIOPWM, byref(cmd), imm = imm)

    # Input
    def get_level(self) -> bool:
        """ このピンへの入力。 """
        self._check_mode(IOController.Mode.LEVEL_INPUT)
        cmd = IODO()
        cmd.address = self.ADDRESS
        cmd.level   = 0
        self.dobot.send_cmd(API.GetIODI, byref(cmd))
        return (cmd.level == 1)

    def get_value(self) -> int:
        """ このピンへのアナログ入力を 0-4095 の値で返します。 """
        self._check_mode(IOController.Mode.AD_INPUT)
        cmd = IOADC()
        cmd.address = self.ADDRESS
        cmd.value = 0
        self.dobot.send_cmd(API.GetIOADC, byref(cmd))
        return cmd.value

# Structures
class IOMultiplexing(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("mode", c_byte)
    ]
class IODO(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("level", c_byte)
    ]
class IOPWM(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("frequency", c_float),
        ("dutyCycle", c_float)
    ]
class IOADC(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("value", c_ushort)
    ]
