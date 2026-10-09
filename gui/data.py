from PySide6.QtCore import Signal, Property, QObject


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