import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from PySide6.QtWidgets import QApplication
from parking_nav.gui import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("地下停车场 IMU/UWB 导航")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
