# Also support direct imports that bypass gui/testing.py.
from qt_runtime import configure_qt_dll_search_path

configure_qt_dll_search_path()

from PySide6.QtWidgets import QWidget
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtCore import QUrl, QObject, Qt
from assets import get_asset


class MapWidget(QQuickWidget):
    _widget = None

    def __init__(self, widget_filepath: str,
                 data_bridge: QObject | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setInitialProperties({"mapClass": data_bridge})
        self.setSource(QUrl.fromLocalFile(get_asset(widget_filepath)))
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setClearColor(Qt.transparent)

    @staticmethod
    def get_widget(data_bridge=None, parent=None):
        if MapWidget._widget is None:
            MapWidget._widget = MapWidget("map.qml", data_bridge, parent)
        return MapWidget._widget

class SpeedWidget(QQuickWidget):
    _widget = None

    def __init__(self, widget_filepath: str,
                 data_bridge: QObject | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setInitialProperties({"speedClass": data_bridge})
        self.setSource(QUrl.fromLocalFile(get_asset(widget_filepath)))
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)
        self.statusChanged.connect(self._report_load_status)
        self._report_load_status(self.status())

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setClearColor(Qt.transparent)

    def _report_load_status(self, status):
        if status == QQuickWidget.Status.Error:
            for error in self.errors():
                print(f"Speed Meter QML error: {error.toString()}", flush=True)
        elif status == QQuickWidget.Status.Ready:
            root = self.rootObject()
            bridge = root.property("speedClass") if root is not None else None
            if bridge is None:
                print("Speed Meter QML loaded without speedClass", flush=True)
            else:
                try:
                    speed = bridge.property("speed")
                    print(f"Speed Meter QML ready (speed={speed})", flush=True)
                except (AttributeError, RuntimeError):
                    print("Speed Meter QML loaded, but speedClass.speed is unavailable", flush=True)

    @staticmethod
    def get_widget(data_bridge=None, parent=None):
        if SpeedWidget._widget is None:
            SpeedWidget._widget = SpeedWidget("speed.qml", data_bridge, parent)
        return SpeedWidget._widget


class BatteryWidget(QQuickWidget):
    _widget = None

    def __init__(self, widget_filepath: str,
                 data_bridge: QObject | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setInitialProperties({"batteryClass": data_bridge})
        self.setSource(QUrl.fromLocalFile(get_asset(widget_filepath)))
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setClearColor(Qt.transparent)

    @staticmethod
    def get_widget(data_bridge=None, parent=None):
        if BatteryWidget._widget is None:
            BatteryWidget._widget = BatteryWidget("battery.qml", data_bridge, parent)
        return BatteryWidget._widget
