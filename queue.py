from ctypes import (byref, c_bool, c_uint64, Structure)

from .base import (DobotServer, AbstractDobotServer, AbstractDobotService)


class QueueServer(AbstractDobotServer):
    """Dobot の Queue コマンドの処理"""
    def __init__(self, dobot: DobotServer):
        self.dobot = dobot
        self._is_executing = False
        self._start = StartExecute(self)
        self._stop = StopExecute(self)
        self._force_stop = ForceStop(self)
        self.clear = ClearQueue(self)
        self.get_cur_idx = GetCurrentIndex(self)

    def is_executing(self):
        """現在 Queue がコマンドを実行する状態か否か"""
        return self._is_executing

    def start_exec(self):
        """Queue を実行状態にする"""
        self._start()
        self._is_executing = True

    def stop_exec(self):
        """Queue を停止状態にする"""
        self._stop()
        self._is_executing = False

    def force_stop(self):
        """Queue を停止状態にする"""
        self._stop()
        self._is_executing = False

    def check_completion(self, cmd_idx: int) -> bool:
        """インデックスからコマンドが終了済みかどうかを判定する

        Args:
            cmd_idx: コマンド送信時の返り値で得られるコマンドのインデックス値。
        """
        return cmd_idx >= self.get_cur_idx()


class AbstractDobotQueueService(AbstractDobotService):
    @staticmethod
    def _check_result(result):
        """Queue コマンド向けのエラー解析

        Raises:
            DobotBufferError
            TimeoutError
            Exception: 公式ドキュメントに記載されていない未知のエラー。
        """
        if result == 0:  # No Error
            return None
        elif result == 1:  # Buffer Full
            raise DobotBufferError("The command queue of Dobot is full.")
        elif result == 2:  # Timeout
            raise TimeoutError(
                "Command does not return, resulting in a timeout.")
        else:  # Unknown
            raise Exception(
                "Unknown Error with starting execute queue commands")
    
    def _send_cmd(self, *params: Structure, imm: bool = False) -> int:
        self._check_connection()
        index = c_uint64(0)
        result = self.COMMAND(*(byref(p) for p in params),
                              c_bool(not imm), byref(index))
        self._check_result(result)
        return index


class DobotBufferError(Exception):
    pass


class _AbstractQueueService(AbstractDobotService):
    """QueueServer のための Dobot のコマンド"""
    def __init__(self, server: QueueServer):
        self.server = server


class StartExecute(_AbstractQueueService):
    """Queue を実行状態にするためのコマンド"""
    COMMAND = DobotServer.Lib.SetQueuedCmdStartExec


class StopExecute(_AbstractQueueService):
    """Queue を停止状態にするためのコマンド"""
    COMMAND = DobotServer.Lib.SetQueuedCmdStopExec


class ForceStop(_AbstractQueueService):
    """Queue を緊急停止"""
    COMMAND = DobotServer.Lib.SetQueuedCmdForceStopExec


class ClearQueue(_AbstractQueueService):
    """Queue の内容を破棄するコマンド"""
    COMMAND = DobotServer.Lib.SetQueuedCmdClear


class GetCurrentIndex(_AbstractQueueService):
    """現在実行中のコマンドのインデックス"""
    COMMAND = DobotServer.Lib.GetQueuedCmdCurrentIndex

    def __call__(self) -> int:
        """現在実行中のコマンドのインデックス"""
        self._check_connection()
        index = c_uint64(0)
        self._check_result(self.COMMAND(byref(index)))
        return index.value


class StartLoopExecute(_AbstractQueueService):
    """(未実装)積まれたコマンドを繰り返し実行する"""
    pass


class StopLoopExecute(_AbstractQueueService):
    """(未実装)積まれたコマンドの繰り返し実行をやめる"""
    pass
