from enum   import Enum

from main import (CommandModule, API)

class IOController(CommandModule):
    class Mode(Enum):
        INVALID = 0
        LEVEL_OUTPUT = 1
        PWM_OUTPUT = 2
        LEVEL_INPUT = 3
        AD_INPUT = 4
        # PULLUP_INPUT = 5
        # PULLDOWN_INPUT = 6
    def __init__(self, dobot):
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
    def __init__(self, dobot, **kwargs):
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
    def __getitem__(self, key):
        return self.PINS_BY_POS[key]
UART = type("UART", Interface, { "PIN_POS": {"E1": 8, "E2": 3, "STOP_KEY": 7} })
SW1  = type("SW1",  Interface, { "PIN_POS": {"VALVE": 1} })
SW2  = type("SW2",  Interface, { "PIN_POS": {"PUMP":  1} })
SW4  = type("SW4",  Interface, { "PIN_POS": {"FAN_12V": 1} })
GPOfBase = type("GPOfBase", Interface, { "PIN_POS": {"REV": 1, "PWM": 2, "ADC": 3} })
GPOfArm  = type("GPOfArm",  Interface, { "PIN_POS": {"PWM": 2, "ADC": 3} })
ANALOG   = type("ANALOG",   Interface, { "PIN_POS": {"Temp": 1} })
class SW3(Interface):
    def __init__(self, dobot, HEAT_12V):
        self.dobot = dobot
        pin = HEAT_12V
        self.HEAT_12V = pin
        self.PINS_BY_POS = {2: pin, 3: pin}

class Pin(CommandModule):
    def __init__(self, dobot, address, volt, permission=[]):
        """
        Parameters
        ----------
        dobot: Dobot
        address: int
        volt: float
        permission: list of IO.Mode
        """
        self.ADDRESS = address
        self.VOLT = volt
        self.PERMISSION = set([IOController.Mode.INVALID]+permission)
        self.dobot = dobot
        self.mode = None
    def config(self, mode, *, imm=False):
        """
        I/O モードの設定

        Parameters
        ----------
        mode: IO.Mode
        imm: bool
            True のとき即座に実行する
        """
        if not mode in self.PERMISSION:
            raise ValueError("this mode is not supported ("+mode._name_+")")
        cmd = IOMultiplexing()
        cmd.address = self.ADDRESS
        cmd.mode = mode.value
        result = self.dobot.queue.send(API.SetIOMultiplexing, byref(cmd), imm=imm)
        pin.set_mode(mode)
        return result
    def check_mode(self, mode):
        if self.mode is not mode:
            raise RuntimeError("This pin is not mode `"+mode._name_+"`")
    # Output
    def level_out(self, level, *, imm=False):
        """
        このピンの出力 ON / OFF

        Parameters
        ----------
        level: bool
        imm: bool
            True のとき即座に実行する
        """
        self.check_mode(IOController.Mode.LEVEL_OUTPUT)
        cmd = IODO()
        cmd.address = self.ADDRESS
        cmd.level   = int(level)
        return self.dobot.queue.send(API.SetIODO, byref(cmd), imm=imm)
    def pwm_out(self, freq, duty, *, imm=False):
        """
        このピンの PWM 出力

        Parameters
        ----------
        freq: float
            周波数(10-1,000,000Hz)
        duty: float
            デューティ比(0-100%)
        imm: bool
            True のとき即座に実行する
        """
        self.check_mode(IOController.Mode.PWM_OUTPUT)
        cmd = IOPWM()
        cmd.address = self.ADDRESS
        cmd.frequency = freq
        cmd.dutyCycle = duty
        return self.dobot.queue.send(API.SetIOPWM, byref(cmd), imm=imm)
    # Input
    def get_level(self):
        """
        このピンへの入力

        Returns
        -------
        level: bool
        """
        self.check_mode(IOController.Mode.LEVEL_INPUT)
        cmd = IODO()
        cmd.address = self.ADDRESS
        cmd.level   = 0
        self.dobot.send_cmd(API.GetIODI, byref(cmd))
        return (cmd.level == 1)
    def get_value(self):
        """
        このピンへのアナログ入力

        Returns
        -------
        value: int
            0-4095
        """
        self.check_mode(IOController.Mode.AD_INPUT)
        cmd = IOADC()
        cmd.address = self.ADDRESS
        cmd.value = 0
        self.dobot.send_cmd(API.GetIOADC, byref(cmd))
        return cmd.value

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
