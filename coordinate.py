from abc import (ABCMeta, abstractmethod)
from collections.abc import Mapping

# methods
def convert_coord(val, relative):
    if not isinstance(val, (Coordinate, dict)):
        raise TypeError("It must be dict or dobot.coordinate.Coordinate.")
    if relative:
        if isinstance(val, RelativeCoordinate):
            return val
        elif all(axis in val for axis in CartesianCoordinateSystem.AXES):
            return CartVector(**val)
        elif all(axis in val for axis in JointCoordinateSystem.AXES):
            return JointVector(**val)
    else:
        if isinstance(val, AbsoluteCoordinate):
            return val
        elif all(axis in val for axis in CartesianCoordinateSystem.AXES):
            return CartCoord(**val)
        elif all(axis in val for axis in JointCoordinateSystem.AXES):
            return JointCoord(**val)
    raise ValueError("It must have keys ('x','y','z','r') or ('j1','j2','j3','j4').")

# Abstract Classes
class MetaCoordinate(ABCMeta):
    def __new__(mcls, *args, **kwargs):
        cls = super().__new__(mcls, *args, **kwargs)
        if not hasattr(cls, "AXES"):
            raise RuntimeError("AXES not found")
        for axis in cls.AXES:
            setattr(cls, axis, property(lambda self: self[axis]))
        return cls

class Coordinate(Mapping, metaclass=MetaCoordinate):
    AXES = tuple()

    def __getitem__(self, key):
        if key in self.AXES:
            return getattr(self, "_"+key)
        else:
            raise KeyError()
    
    def __iter__(self):
        return iter(self.AXES)
    
    def __len__(self):
        return len(self.AXES)
    
    @abstractmethod
    def __init__(self, *args):
        if len(args) != len(self.AXES):
            message  = self.__class__.__name__+"() takes exactly "
            message += str(len(self.AXES))+" axis ("+str(len(args))+" given)"
            raise TypeError(message)
        while args[-1] < -180: args[-1] += 360
        while args[-1] > +180: args[-1] -= 360
        for axis, arg in zip(self.AXES, args):
            setattr(self, "_" + axis, arg)
    
    @abstractmethod
    def is_relative(self):
        pass
    
    #@abstractmethod
    def __add__(self, other):
        pass
    #@abstractmethod
    def __sub__(self, other):
        pass
    #@abstractmethod
    def __mul__(self, val):
        pass
    #@abstractmethod
    def __div__(self, val):
        pass

class AbsoluteCoordinate(Coordinate):
    #@abstractmethod
    def is_valid(self):
        """ 可動域の中かどうかを返す関数 """
        pass
    
    def is_relative(self):
        return False

    def infiltrate(self, target, labels={}):
        """
        API用の構造体に自分の値を挿入

        Parameters
        ----------
        target: ctypes.Structure
            挿入するための構造体
        labels: dict
            軸の名称をキーとして、構造体のラベルを代入
        """
        for axis in self.AXES:
            if axis in labels:
                setattr(target, labels[axis], self[axis])
            else:
                setattr(target, axis, self[axis])
        return target

class RelativeCoordinate(Coordinate):
    def is_relative(self):
        return True

class CartesianCoordinateSystem(Coordinate):
    AXES = ("x", "y", "z", "r")

    def __init__(self, x, y, z, r=0):
        super().__init__(x, y, z, r)

    def scalar_product(self, other):
        pass


class JointCoordinateSystem(Coordinate):
    AXES = ("j1", "j2", "j3", "j4")

    def __init__(self, j1, j2, j3, j4):
        super().__init__(j1, j2, j3, j4)


# Partical Classes

class CartCoord(CartesianCoordinateSystem, AbsoluteCoordinate):
    def __init__(self, x, y, z, r, check=False):
        """
        Parameters
        ----------
        x: float
            肩の関節の中心を原点とし、前方を正とする座標軸のエンドエフェクタのモーター付け根部分の位置。単位はmm。
        y: float
            アームから見て左側を正とする座標軸。
        z: float
            上方を正とする座標軸。
        r: float
            上から見て反時計回りを正とするエンドエフェクタの角度。単位は度数法。
        check: bool
            代入された値が可動域内かどうかを試験する（未実装）
        """
        super().__init__(x, y, z, r)
        if check and not self.is_valid():
            raise ValueError("This position is invalid")

class CartVector(CartesianCoordinateSystem, RelativeCoordinate):
    pass

class JointCoord(JointCoordinateSystem, AbsoluteCoordinate):
    def __init__(self, j1, j2, j3, j4, check=False):
        """
        Parameters
        ----------
        j1: float
            肩の関節の中心を原点とし、前方を正とする座標軸のエンドエフェクタのモーター付け根部分の位置。単位はmm。
        j2: float
            アームから見て左側を正とする座標軸。
        j3: float
            上方を正とする座標軸。
        j4: float
            上から見て反時計回りを正とするエンドエフェクタの角度。単位は度数法。
        check: bool
            代入された値が可動域内かどうかを試験する（未実装）
        """
        super().__init__(j1, j2, j3, j4)
        if check and not self.is_valid():
            raise ValueError("This position is invalid")

class JointVector(JointCoordinateSystem, RelativeCoordinate):
    pass
