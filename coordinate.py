import ctypes

from abc import (ABCMeta, abstractmethod)
from collections import abc
from typing import (Optional, Mapping, Iterator, SupportsFloat)


# Abstract Classes
class MetaCoordinate(ABCMeta):
    """ 座標型が軸を持つことを保証するためのメタクラスです。 """
    def __new__(mcls, *args, **kwargs):
        cls = super().__new__(mcls, *args, **kwargs)
        if not hasattr(cls, "AXES"):
            raise RuntimeError("AXES not found")
        for axis in cls.AXES:
            setattr(cls, axis, property(lambda self: self[axis]))
        return cls


class Coordinate(abc.Mapping, metaclass=MetaCoordinate):
    """ 座標を表す抽象クラスです。 """
    AXES = tuple()

    def __getitem__(self, key: str) -> float:
        """ 軸を表す文字列を代入すると、その軸のパラメータを返します。 """
        if key in self.AXES:
            return getattr(self, "_" + key)
        else:
            raise KeyError()

    def __iter__(self) -> Iterator[str]:
        return iter(self.AXES)

    def __len__(self) -> int:
        return len(self.AXES)

    @abstractmethod
    def __init__(self, *args):
        if len(args) != len(self.AXES):
            message = self.__class__.__name__ + "() takes exactly "
            message += str(len(self.AXES)) + " axis ("
            message += str(len(args)) + " given)"
            raise TypeError(message)
        while args[-1] < -180: args[-1] += 360
        while args[-1] > +180: args[-1] -= 360
        for axis, arg in zip(self.AXES, args):
            setattr(self, "_" + axis, arg)

    @abstractmethod
    def is_relative(self) -> bool:
        """ Dobot の原点を中心とした絶対座標かそうでないかを bool で返します。 """
        pass

    # @abstractmethod
    def __add__(self, other: Mapping[str, SupportsFloat]) -> 'Coordinate':
        pass

    # @abstractmethod
    def __sub__(self, other: Mapping[str, SupportsFloat]) -> 'Coordinate':
        pass

    # @abstractmethod
    def __mul__(self, val: SupportsFloat) -> 'Coordinate':
        pass

    # @abstractmethod
    def __div__(self, val: SupportsFloat) -> 'Coordinate':
        pass


class AbsoluteCoordinate(Coordinate):
    """ Dobot の原点を中心とした絶対座標であることを表すクラスです。 """

    # @abstractmethod
    def is_valid(self) -> bool:
        """ 自分自身が Dobot の可動域の中かどうかを返す関数。 """
        pass

    def is_relative(self) -> bool:
        return False

    def infiltrate(
            self, target: ctypes.Structure,
            labels: Mapping[str, Optional[str]] = {}) -> ctypes.Structure:
        """ 内部でAPIを送信するためのメソッドです。構造体を引数に取り、自身の値をその構造体に代入します。

        labels は Coordinate オブジェクトでの軸の名称から構造体に代入したい要素名への dict です。
        None を指定するとその軸の値が無視されます。
        """
        for axis in self.AXES:
            if axis in labels:
                setattr(target, labels[axis], float(self[axis]))
            elif self[axis] is not None:
                setattr(target, axis, float(self[axis]))
        return target


class RelativeCoordinate(Coordinate):
    """ 相対座標であることを表すクラスです。 """
    def is_relative(self):
        return True


class CartesianCoordinateSystem(Coordinate):
    """ デカルト座標系を表すクラスです。先端部分の回転角度も持ちます。

    x は Dobot の正面（土台の各種配線のある部分の反対側）を正とします。
    y はこの正面を向いて左側を正とします。z は上向きを正とします。
    x, y, z の単位はそれぞれ mm です。
    r は上から見て反時計回りを正とするエンドエフェクタの角度です。度数法で表します。
    """
    AXES = ("x", "y", "z", "r")

    def __init__(self, x, y, z, r=0.0) -> 'CartesianCoordinateSystem':
        if type(self) is CartesianCoordinateSystem:
            raise TypeError("You can't create an instance of "
                            "`CoordinateSystem` class.")
        super().__init__(x, y, z, r)

    def scalar_product(self, other):
        pass


class JointCoordinateSystem(Coordinate):
    """ 各関節の角度で示される座標系であることを表すクラスです。 """
    AXES = ("j1", "j2", "j3", "j4")

    def __init__(self, j1, j2, j3, j4) -> 'JointCoordinateSystem':
        if type(self) is JointCoordinateSystem:
            raise TypeError("You can't create an instance of "
                            "`CoordinateSystem` class.")
        super().__init__(j1, j2, j3, j4)


# Partical Classes
class CartCoord(CartesianCoordinateSystem, AbsoluteCoordinate):
    """ アームの軸の中心を原点とした、3次元デカルト座標と先端部分の回転角度です。

    原点は Dobot の左右に突き出したモーター部分の中点、
    座標の指し示す場所は純正エンドエフェクタを付けた場合のエンドエフェクタの取り付け部分になります。
    各軸の詳細は CartesianCoordinateSystem の説明参照のこと。
    """
    def __init__(self, x: SupportsFloat, y: SupportsFloat, z: SupportsFloat,
                 r: SupportsFloat = 0.0, check: bool = False):
        """ check を True にすると指定した座標が可動域に収まっているかどうかが判定されます。（未実装） """
        super().__init__(x, y, z, r)
        if check and not self.is_valid():
            raise ValueError("This position is invalid")


class CartVector(CartesianCoordinateSystem, RelativeCoordinate):
    """ 3次元デカルト座標で相対座標を示すものです。

    各軸の詳細は CartesianCoordinateSystem の説明参照のこと。
    """
    pass


class JointCoord(JointCoordinateSystem, AbsoluteCoordinate):
    """ 各関節の角度で、アームの姿勢を表したものです。

    各関節の詳細は JointCoordinateSystem の説明参照のこと。
    """
    def __init__(self, j1: SupportsFloat, j2: SupportsFloat,
                 j3: SupportsFloat, j4: SupportsFloat, check: bool = False):
        """ check を True にすると指定した座標が可動域に収まっているかどうかが判定されます。（未実装） """
        super().__init__(j1, j2, j3, j4)
        if check and not self.is_valid():
            raise ValueError("This position is invalid")


class JointVector(JointCoordinateSystem, RelativeCoordinate):
    """ 各関節の相対的な角度を示すものです。

    各関節の詳細は CartesianCoordinateSystem の説明参照のこと。
    """
    pass


# methods
def convert_coord(point: Mapping[str, SupportsFloat], relative: bool)\
        -> Coordinate:
    """ point を適切な座標クラスに変換するための関数です。 """
    if not isinstance(point, Mapping[str, SupportsFloat]):
        raise TypeError("The point must be an object shows any coordinate.")
    if relative:
        if isinstance(point, RelativeCoordinate):
            return point
        elif all(axis in point for axis in CartesianCoordinateSystem.AXES):
            return CartVector(**point)
        elif all(axis in point for axis in JointCoordinateSystem.AXES):
            return JointVector(**point)
    else:
        if isinstance(point, AbsoluteCoordinate):
            return point
        elif all(axis in point for axis in CartesianCoordinateSystem.AXES):
            return CartCoord(**point)
        elif all(axis in point for axis in JointCoordinateSystem.AXES):
            return JointCoord(**point)
    raise ValueError("It must have keys ('x','y','z','r') "
                     "or ('j1','j2','j3','j4').")
