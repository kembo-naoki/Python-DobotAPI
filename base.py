import functools
from os import path
from abc import (ABCMeta, abstractmethod)
from ctypes import (cdll, create_string_buffer)
from typing import (Callable, Dict)


_CUR_DIR = path.dirname(path.abspath(__file__))
API = cdll.LoadLibrary(_CUR_DIR + "/libDobotDll.so.1.0.0")


class DobotClient(metaclass=ABCMeta):
    @abstractmethod
    def __init__(self, logger=None):
        """ インスタンスを作成した時点ではまだ接続はしません。 """
        pass

    # with 文のサポート
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    # コマンドそのものに関するメソッド
    @abstractmethod
    def send_command(self, cmd: API._FuncPtr, args: tuple = tuple()):
        """ コマンドを送信する """
        pass

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
        result = API.ConnectDobot(sz_para,  baud_rate,  fw_type,  version)
        if result == 0:  # No Error
            self.is_connected = True
            return True
        elif result == 1:
            raise DobotConnectionError(
                "connection error with first connecting")
        elif result == 2:
            raise DobotTimeout("timeout error with first connecting")
        else:
            raise RuntimeError("unknown error with first connecting")

    def disconnect(self):
        """ 切断 """
        API.DisconnectDobot()
        self.is_connected = False

    def force_stop(self):
        """ 実行中のコマンドも含めて緊急停止し、キューにコマンドを積めなくなる。

        `Dobot.queue_clear()`, `Dobot.queue_start()`, `Dobot.queue_pause()`
        のいずれかのメソッドで再びキューにコマンドを積めるようになる。
        """
        self.stop_flg = True
        self.send_command(API.SetQueuedCmdForceStopExec)


# prepare for command classes
def setting_method(func: Callable):
    """ このメソッドを設定用メソッドとする """
    func.setting_method = True
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
            if (isinstance(f, Callable)
                and getattr(f, "setting_method", False))
        )
        return new_cls


class DobotCommand(metaclass=_MetaCommand):
    """ Dobot の各機能を表す抽象クラス。 """
    SETTING_METHODS = tuple()

    def __init__(self, dobot: DobotClient):
        self.dobot = dobot
        self._progress_of_settings = \
            dict([name, False] for name in self.SETTING_METHODS)

    def progress_of_settings(self) -> Dict[str, bool]:
        """ そのコマンドのための設定の完了状況を示すメソッド。

        設定用のメソッド名をキーとし、設定の完了状況を`bool`で示した`dict`が返されます。
        """
        return self._progress_of_settings.copy()


# Errors
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
