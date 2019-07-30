from os  import path
from abc import (ABCMeta, abstractmethod)
from ctypes import cdll 

_CUR_DIR = path.dirname( path.abspath( __file__ ) )
API = cdll.LoadLibrary(_CUR_DIR + "/libDobotDll.so.1.0.0")

class CommandModule(metaclass=ABCMeta):
    """ Dobot の機能毎のクラス """
    @abstractmethod
    def __init__(self, dobot):
        self.dobot = dobot
        self.check_list = {}
        self.requirements = ()
    
    def check_settings(self, *opts):
        """ このコマンドが送信可能か否か """ 
        req_list = self.requirements + opts
        message = None
        for req in req_list:
            if isinstance(req, tuple):
                if not any(self.check_list[k] for k in req):
                    message  = "need setting by at least one method of ("
                    message += ", ".join("set_"+name for name in req) + ")"
            elif not self.check_list[req]:
                message = "need setting by `" + req + "` method"
        if message is not None:
            raise RuntimeError(message)
        return True