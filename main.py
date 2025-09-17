import sys
from PyQt6.QtWidgets import QApplication
from pid import PID
from ekf import KalmanFilter
from gui import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(PID, KalmanFilter)
    win.show()
    sys.exit(app.exec())