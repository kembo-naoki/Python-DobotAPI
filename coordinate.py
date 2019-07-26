from abc import (ABCMeta, abstractmethod)

# Abstract Classes

class Coordinate(metaclass=ABCMeta):
    def __setattr__(self, name, value):
        raise TypeError("Class `Coordinate` does not support attribute assignment.")
    
    @abstractmethod
    def __init__(self, *args, **kwargs):
        raise TypeError("This class is abstract class.")

    @abstractmethod
    def __add__(self, other):
        pass
    @abstractmethod
    def __sub__(self, other):
        pass
    @abstractmethod
    def __mul__(self, val):
        pass
    @abstractmethod
    def __div__(self, val):
        pass

class AbsoluteCoordinate(metaclass=ABCMeta):
    @abstractmethod
    def validate(self):
        """ 可動域のチェック """
        pass
    @abstractmethod
    def infiltrate(self, target, labels):
        """ API用の構造体に自分の値を挿入 """
        pass

class RelativeCoordinate(metaclass=ABCMeta):
    pass

class CartesianCoordinate(Coordinate, metaclass=ABCMeta):
    def __init__(self, x, y, z, r):
        object.__setattr__(self, "x", x)
        object.__setattr__(self, "y", y)
        object.__setattr__(self, "z", z)
        if   r < -180: r += 360
        elif r >  180: r -= 360
        object.__setattr__(self, "r", r)

    def scalar_product(self, other):
        pass


class JointCoordinate(Coordinate, metaclass=ABCMeta):
    def __init__(self, j1, j2, j3, j4):
        object.__setattr__(self, "j1", j1)
        object.__setattr__(self, "j2", j2)
        object.__setattr__(self, "j3", j3)
        if   j4 < -180: j4 += 360
        elif j4 >  180: j4 -= 360
        object.__setattr__(self, "j4", j4)


# Partical Classes

class CartesianAbsoluteCoordinate(CartesianCoordinate, AbsoluteCoordinate):
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
        if check:
            self.validate

    def infiltrate(self, target, x="x", y="y", z="z", r="r"):
        object.__setattr__(target, x, self.x)
        object.__setattr__(target, y, self.y)
        object.__setattr__(target, z, self.z)
        object.__setattr__(target, r, self.r)
        return target

class CartesianRelativeCoordinate(CartesianCoordinate, RelativeCoordinate):
    pass

class JointAbsoluteCoordinate(JointCoordinate, AbsoluteCoordinate):
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
        if check and not self.validate():
            raise ValueError("This position is invalid")
        super().__init__(j1, j2, j3, j4)

    def validate(self):
        """ 可動域の中かどうかを返す関数 """
        pass

    def infiltrate(self, target, j1="j1", j2="j2", j3="j3", j4="j4"):
        object.__setattr__(target, j1, self.j1)
        object.__setattr__(target, j2, self.j2)
        object.__setattr__(target, j3, self.j3)
        object.__setattr__(target, j4, self.j4)
        return target

class JointRelativeCoordinate(JointCoordinate, RelativeCoordinate):
    pass

