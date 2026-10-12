"""Windows Qt DLL search-path setup for PySide's dynamically loaded plugins."""

import os
from pathlib import Path


# Keep the handles alive: Windows removes each search directory when its
# handle is closed, while the QML engine loads plugins later during startup.
_dll_directory_handles = []


def configure_qt_dll_search_path() -> None:
    if os.name != "nt" or _dll_directory_handles:
        return

    # find_spec locates the installed package without importing PySide6.
    import importlib.util

    spec = importlib.util.find_spec("PySide6")
    if spec is None or not spec.submodule_search_locations:
        return

    package_dir = Path(next(iter(spec.submodule_search_locations))).resolve()
    shapes_dir = package_dir / "qml" / "QtQuick" / "Shapes"

    for directory in (package_dir, package_dir / "Qt" / "bin", shapes_dir):
        if directory.is_dir():
            _dll_directory_handles.append(os.add_dll_directory(str(directory)))

