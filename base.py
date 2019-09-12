from os import path
from ctypes import (cdll, create_string_buffer)
from typing import (Optional, List)

from abstract_classes import (Server, Service)


class DobotServer(Server):
    _CUR_DIR = path.dirname(path.abspath(__file__))
    Lib = cdll.LoadLibrary(_CUR_DIR + "/libDobotDll.so.1.0.0")

    def __init__(self):
        self._is_connected = False
        self.search = DobotSearcher(self)
        self.connect = DobotConnector(self)
        self.firmware_type = "Nothing"
        self.firmware_version = "Nothing"
        self.disconnect = DobotDisconnector(self)

    def is_started(self):
        return self._is_connected

    # with 文のサポート
    def __enter__(self, port: Optional(str) = None) -> 'DobotServer':
        self.connect(port)
        return self

    def __exit__(self, *_):
        self.disconnect()


class DobotService(Service):
    def __init__(self, server: DobotServer):
        self.server = server


class DobotSearcher(DobotService):
    MAX_LEN = 128

    def __call__(self) -> List[str]:
        buf_result = create_string_buffer(self.MAX_LEN)
        num = self.server.Lib.SearchDobot(buf_result, self.MAX_LEN)
        if num == 0:
            return []
        return buf_result.value.decode("utf-8").split(' ')


class DobotConnector(DobotService):
    def __init__(self, server: DobotServer, default_port: str = "",
                 baud_rate: int = 115200, max_str_len: int = 100,
                 encode_type: str = "utf-8"):
        self.server = server

        self.port = default_port
        self.baud_rate = baud_rate
        self.max_str_len = max_str_len
        self.encode_type = encode_type

    def __call__(self, port: Optional(str) = None) -> dict:
        """ 接続 """
        if port is not None:
            self.port = port

        port_name = create_string_buffer(
            self.port.encode(self.encode_type), self.max_str_len)
        fw_type = create_string_buffer(self.max_str_len)
        version = create_string_buffer(self.max_str_len)

        result = self.server.Lib.ConnectDobot(
            port_name, self.baud_rate, fw_type, version)

        if result == 0:  # No Error
            self.server._is_connected = True
            fw_type = fw_type.value.decode(self.encode_type)
            version = version.value.decode(self.encode_type)
            self.server.firmware_type = fw_type
            self.server.firmware_version = version
            return {
                "firmware_type": fw_type, "firmware_version": version
            }
        elif result == 1:
            raise ConnectionError(
                "Connection Error with connecting Dobot")
        elif result == 2:
            raise TimeoutError("Timeout Error with connecting Dobot")
        else:
            raise RuntimeError("Unknown Error with connecting Dobot")


class DobotDisconnector(DobotService):
    def __call__(self) -> None:
        """ 切断 """
        self.server.API.DisconnectDobot()
        self.server._is_connected = False
        self.server.firmware_type = "Nothing"
        self.server.firmware_version = "Nothing"
