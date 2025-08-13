# main.py

from PyQt5.QtWidgets import QApplication
from mainwindow import MainWindow
import sys

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1200, 900)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()