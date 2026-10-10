from PySide6.QtCore import Signal, Property, QObject


class MapData(QObject):
    position_changed = Signal()

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        super().__init__()
        self._x = x
        self._y = y
        self._theta = theta

    @Property(float, notify=position_changed)
    def x(self):
        return self._x

    @x.setter
    def x(self, val: float):
        if self._x != val:
            self._x = val
            self.position_changed.emit()

    @Property(float, notify=position_changed)
    def y(self):
        return self._y

    @y.setter
    def y(self, val: float):
        if self._y != val:
            self._y = val
            self.position_changed.emit()

    @Property(float, notify=position_changed)
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, val: float):
        if self._theta != val:
            self._theta = val
            self.position_changed.emit()

class SpeedData(QObject):
    speed_changed = Signal()

    def __init__(self, speed: float = 0.0):
        super().__init__()
        self._speed = speed

    @Property(float, notify=speed_changed)
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed: float):
        if speed < 0.0:
            speed = 0.0
        if speed > 1.0:
            speed = 1.0

        if self._speed != speed:
            self._speed = speed
        self.speed_changed.emit()


class BatteryData(QObject):
    level_changed = Signal()

    def __init__(self, level: int = 0):
        super().__init__()
        self._level = level

    @Property(int, notify=level_changed)
    def level(self):
        return self._level

    @level.setter
    def level(self, level: int):
        if level < 0:
            level = 0
        if level > 100:
            level = 100

        if self._level != level:
            self._level = level
        self.level_changed.emit()