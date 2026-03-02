import sys

from PySide6.QtWidgets import QApplication

from rift.ui.main import MainWindow


app = QApplication(sys.argv)
widget = MainWindow()
widget.show()
sys.exit(app.exec())
