
import sys
from pathlib import Path

# Add the project root and GUI folder before importing project modules.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
GUI_DIR = Path(__file__).resolve().parent

for directory in (PROJECT_ROOT, GUI_DIR):
    directory_str = str(directory)
    if directory_str not in sys.path:
        sys.path.insert(0, directory_str)

# Configure Qt DLL paths before importing PySide6.
from qt_runtime import configure_qt_dll_search_path

configure_qt_dll_search_path()

from PySide6.QtWidgets import QApplication
from window import Window


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = Window()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
