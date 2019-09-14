import ctypes

from abc import (ABCMeta, abstractmethod)
from collections import abc
from typing import (Optional, Iterator, Union)


class AbstractCoordinate(abc.Mapping, metaclass=ABCMeta):
    """座標を表す抽象クラスです
    
    Attributes:
        AXES: 軸リスト
    """
    AXES = ()

    @abstractmethod
    def __init__(self, *args):
        pass

    @abstractmethod
    def __getitem__(self, key: str) -> float:
        pass

    def __iter__(self) -> Iterator[str]:
        return iter(self.AXES)

    def __len__(self) -> int:
        return len(self.AXES)


class CartCoord(AbstractCoordinate):
    """アームの先端サーボの回転軸付け根の位置のデカルト座標系と回転角度

    Attributes:
        x: 左右のサーボの回転軸の中心点を0として正面方向を正とする。(mm)
        y: 左右のサーボの回転軸の中心点を0としてx軸の方向に対して左側を正とする。(mm)
        z: 左右のサーボの回転軸を0として、鉛直上方を正とする。(mm)
        r: アーム先端サーボの回転角。上空から見下ろして反時計回りを正とする度数法。(mm)
    """
    AXES = ("x", "y", "z", "r")

    def __init__(self, x: float = 0, y: float = 0,
                 z: float = 0, r: float = 0.0):
        """アームの先端サーボの回転軸付け根の位置のデカルト座標系と回転角度

        Args:
            x: 左右のサーボの回転軸の中心点を0として正面方向を正とする。(mm)
            y: 左右のサーボの回転軸の中心点を0としてx軸の方向に対して左側を正とする。(mm)
            z: 左右のサーボの回転軸を0として、鉛直上方を正とする。(mm)
            r: アーム先端サーボの回転角。上空から見下ろして反時計回りを正とする度数法。(mm)
        """
        self.x, self.y, self.z = x, y, z
        while r < -180: r += 360
        while r > +180: r -= 360
        self.r = r

    def __getitem__(self, key: str) -> float:
        if key == "x":
            return self.x
        elif key == "y":
            return self.y
        elif key == "z":
            return self.z
        elif key == "r":
            return self.r
        else:
            raise KeyError


class JointCoord(AbstractCoordinate):
    """各関節のサーボの角度(度数法)を表す

    Attributes:
        j1: 土台のサーボの回転角。
        j2: 肩部分のサーボの回転角。
        j3: 肘部分のサーボの回転角。
        j4: 先端部分のサーボの回転角。CartCoord の r と同じ。
    """
    AXES = ("j1", "j2", "j3", "j4")

    def __init__(self, j1: float = 0, j2: float = 0,
                 j3: float = 0, j4: float = 0):
        """各関節のサーボの角度(度数法)を表す

        Args:
            j1: 土台のサーボの回転角。
            j2: 肩部分のサーボの回転角。
            j3: 肘部分のサーボの回転角。
            j4: 先端部分のサーボの回転角。CartCoord の r と同じ。
        """
        self.j1, self.j2 = j1, j2
        self.j3, self.j4 = j3, j4

    def __getitem__(self, key: Union[int, str]) -> float:
        if key == "j1" or key == 0:
            return self.j1
        elif key == "j2" or key == 1:
            return self.j2
        elif key == "j3" or key == 2:
            return self.j3
        elif key == "j4" or key == 3:
            return self.j4
        else:
            raise KeyError
