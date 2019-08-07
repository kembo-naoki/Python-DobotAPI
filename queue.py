from time import (sleep)
from ctypes import (byref, c_uint64)

from .base import (API, DobotClient, DobotCommand)


class QueueController(DobotCommand):
    LIM_COMMAND = 5
    @property
    def last_index(self):
        return self._last_index

    def __init__(self, dobot: DobotClient):
        super().__init__(dobot)
        self._last_index = -1

    def send(self, cmd: API._FuncPtr,
             args: tuple = tuple(), imm: bool = False) -> int:
        """ コマンドを Dobot のキューに積む。

        キューに積まず即座に実行したい場合は、`imm`を`True`にしてください。
        """
        cmd_index = c_uint64(0)
        mode = 0 if imm else 1
        self.dobot.send_command(cmd, args=(*args, mode, byref(cmd_index)))
        idx = cmd_index.value
        self._last_index = idx
        return self._last_index

    def clear(self):
        """ キューに積まれたコマンドをクリア """
        self.dobot.send_command(API.SetQueuedCmdClear)

    def start(self):
        """ キューに積まれたコマンドの実行を開始 """
        self.dobot.send_command(API.SetQueuedCmdStartExec)

    def pause(self):
        """ キューに積まれたコマンドを停止（実行中のコマンドは止まらない） """
        self.dobot.send_command(API.SetQueuedCmdStopExec)

    def get_current_index(self) -> int:
        """ 現在まで実行完了済のキューインデックス """
        queued_cmd_index = c_uint64(0)
        counter = 0
        while True:
            self.dobot.send_command(API.GetQueuedCmdCurrentIndex,
                                    args=(byref(queued_cmd_index),))
            index = queued_cmd_index.value
            if index <= self._last_index:
                break

            counter += 1
            if counter > self.LIM_COMMAND:
                raise TimeoutError(
                    "timeout error with getting current command")
            sleep(0.5)
        return index
