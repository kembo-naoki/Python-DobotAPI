from abc import (ABCMeta, abstractmethod)


class Server(metaclass=ABCMeta):
    """ 関連性のある最低限の Service をとりまとめるためのもの """
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def is_started(self):
        """ 連携する Server が生きてるかどうかをチェック """
        pass


class AsyncServer(Server):
    """ 非同期的に Service を提供するためのもの """
    @abstractmethod
    def __init__(self):
        """ 初期化だけではサービスの提供は始まらない """
        self._is_started = False

    def is_started(self):
        return self._is_started

    @abstractmethod
    def start(self):
        """ 接続 & スレッドの開始 """
        pass

    @abstractmethod
    def stop(self):
        """ 切断 & 停止 """
        pass


class Service(metaclass=ABCMeta):
    """ 各コマンドを司るクラス """
    def __init__(self, server: Server):
        self.server = server

    @abstractmethod
    def __call__(self):
        pass
