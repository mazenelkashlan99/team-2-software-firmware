# Register Qt DLL search paths before importing PySide6 or creating a QML engine.
from qt_runtime import configure_qt_dll_search_path

configure_qt_dll_search_path()

from PySide6.QtWidgets import QApplication
from window import Window

def main():
    app = QApplication()
    window = Window()
    app.setStyle("Fusion")
    app.exec()

if __name__ == "__main__":
    main()
