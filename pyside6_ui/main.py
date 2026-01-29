# This Python file uses the following encoding: utf-8
import sys
from datetime import datetime
import logging
# from PySide6.QtCore import QThread
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtWebEngineWidgets import QWebEngineView

from ui_main import Ui_MainWindow
from Handlers.joystick_handler import JoystickHandler
from Handlers.bot_viz import html_bit

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Set Up UI from generated UI form
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        #Roll global functions into UI object
        self.ui.term_log = self.term_log

        self.ui.term_log("Welcome to R.I.F.T. Control!")

        # Add in Web Engine View to display Plotly HTML object for robot
        self.ui.bot_viz = QWebEngineView()
        self.ui.bot_viz.setHtml(html_bit)
        self.ui.bot_viz.setHtml('<img src="https://i.natgeofe.com/k/a2ddd1ed-c7b5-4825-8223-203ecbb31956/stick-insect-branch.jpg?wp=1&w=1084.125&h=609">')
        self.ui.bot_viz.show()
        self.ui.Full_Splitter.insertWidget(0, self.ui.bot_viz)
        self.joystick_handler = JoystickHandler(self.ui)

    def term_log(self, text):
        time = datetime.now().strftime("%m/%d/%y %H:%M:%S")
        self.ui.term.appendPlainText(f"{time} - {text}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
