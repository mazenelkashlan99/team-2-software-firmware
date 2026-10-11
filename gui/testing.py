from PySide6.QtWidgets import QWidget, QApplication, QMainWindow
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtCore import QUrl, QObject
from window import Window

def main():
    app = QApplication()
    window = Window()
    app.setStyle("Fusion")
    app.exec()

if __name__ == "__main__":
    main()