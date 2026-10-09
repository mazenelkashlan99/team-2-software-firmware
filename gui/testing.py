from PySide6.QtWidgets import QWidget, QApplication, QMainWindow
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtCore import QUrl, QObject
from assets import get_asset


class StatusWidget(QQuickWidget):
    def __init__(self, widget_filepath: str,
                 data_bridge: QObject | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)

        self.setInitialProperties({"dataClass": data_bridge})

        self.setSource(QUrl.fromLocalFile(get_asset(widget_filepath)))

        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

def main():
    app = QApplication()
    widget = StatusWidget("speed.qml")
    window = QMainWindow()
    window.setCentralWidget(widget)
    window.show()
    app.exec()

if __name__ == "__main__":
    main()