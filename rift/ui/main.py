import sys, pathlib
sys.path.append(str(pathlib.Path.cwd().parent.parent))
from datetime import datetime

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication, QMainWindow
# from PySide6.QtWebEngineWidgets import QWebEngineView
# from PySide6 import QtAsyncio

from ui_main import Ui_Control
from Handlers.joystick_handler import JoystickHandler
from Handlers.vis_handler import SimWindow

from rift import rover
from rift.robot import InverseKinematicsError
from rift.arraytypes import Matrix

from rift.steps import Command, Mode

class MainWindow(QMainWindow): #referenced as widget by sim window class
    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_Control()
        self.ui.setupUi(self)
        self.ui.term_log = self.term_log
        self.ui.term_log("Welcome to R.I.F.T. Control!")

        self.cmd_state = Command(Mode.offline, 0, 0, 0, 0)

        self.joystick_handler = JoystickHandler(self.ui)
        self.vis_handler = SimWindow(self.cmd_state)

        self.ui.selector_label.setVisible(False)
        self.ui.selector.setVisible(False)

        self.ui.sim_toggle.clicked.connect(self.toggle_sim)
        self.ui.sim_label.clicked.connect(self.open_sim)
        self.ui.selector.valueChanged.connect(self.update_item)

        self.ui.forward.pressed.connect(lambda: self.cmd_update(1, 0, 0))
        # self.ui.forward.pressed.connect(self.cleanup)
        self.ui.backward.pressed.connect(lambda: self.cmd_update(-1, 0, 0))
        self.ui.left.pressed.connect(lambda: self.cmd_update(0, -1, 0))
        self.ui.right.pressed.connect(lambda: self.cmd_update(0, 1, 0))
        self.ui.del_right.pressed.connect(lambda: self.cmd_update(0, 0, 1))
        self.ui.del_left.pressed.connect(lambda: self.cmd_update(0, 0, -1))

        self.ui.forward.released.connect(lambda: self.cmd_update(-1, 0, 0))
        self.ui.backward.released.connect(lambda: self.cmd_update(1, 0, 0))
        self.ui.left.released.connect(lambda: self.cmd_update(0, 1, 0))
        self.ui.right.released.connect(lambda: self.cmd_update(0, -1, 0))
        self.ui.del_right.released.connect(lambda: self.cmd_update(0, 0, -1))
        self.ui.del_left.released.connect(lambda: self.cmd_update(0, 0, 1))

        self.ui.crawling.clicked.connect(lambda: self.mode_select('crawling'))
        self.ui.node_control.clicked.connect(lambda: self.mode_select('node_control'))
        self.ui.calibration.clicked.connect(lambda: self.mode_select('calibration'))

    def toggle_sim(self):
        if not self.vis_handler.view_live:
            self.greenify(self.ui.sim_label)
            self.ui.sim_label.setText("Click to Open")
            self.ui.sim_toggle.setText("Kill Simulation")
            self.ui.term_log("Simulation Initialized")
            self.vis_handler.start_sim()
        else:
            self.redify(self.ui.sim_label)
            self.ui.sim_label.setText("Simulation Offline")
            self.ui.sim_toggle.setText("Kill Simulation")
            self.ui.term_log("Simulation Closed")
            self.vis_handler.kill_sim()

    def update_item(self):
        self.cmd_state.item = self.ui.selector.value()

    def open_sim(self):
        if self.vis_handler.view_live:
            self.vis_handler.show()

    def mode_select(self, mode):
        self.ui.left.setEnabled(True)
        self.ui.del_left.setEnabled(True)
        self.ui.right.setEnabled(True)
        self.ui.del_right.setEnabled(True)

        if mode == "crawling":
            self.plainify_modes()
            self.greenify(self.ui.crawling)
            self.cmd_state.mode = Mode.crawling
            self.cmd_state.item = 0
            self.ui.selector_label.setVisible(False)
            self.ui.selector.setVisible(False)
        elif mode == "node_control":
            self.plainify_modes()
            self.greenify(self.ui.node_control)
            self.cmd_state.mode = Mode.node_control
            self.cmd_state.item = self.ui.selector.value()
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Node")
            self.ui.selector.setVisible(True)
        elif mode == "calibration":
            self.plainify_modes()
            self.greenify(self.ui.calibration)
            self.cmd_state.mode = Mode.calibration
            self.cmd_state.item = self.ui.selector.value()
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Roller")
            self.ui.selector.setVisible(True)
            self.ui.left.setEnabled(False)
            self.ui.del_left.setEnabled(False)
            self.ui.right.setEnabled(False)
            self.ui.del_right.setEnabled(False)
        self.ui.term_log(f"Control Mode switched to {mode.replace('_',' ')}")

    def cmd_update(self, x: float, y: float, z: float):
        self.cmd_state.x += x
        self.cmd_state.y += y
        self.cmd_state.z += z
        print(f"X: {self.cmd_state.x}, Y: {self.cmd_state.y}, Z: {self.cmd_state.z}")

    def cleanup(self):
        print("attempting cleanup")
        try:
            self.joystick_handler.js_thread.requestInterruption()
            self.joystick_handler.js_thread.quit()
            self.joystick_handler.js_worker.deleteLater()
        except:
            print("No Joystick to kill")
        try:
            self.vis_handler.thread.requestInterruption()
            print("Sim killed")
        except:
            print("No Sim to kill")

    #overwriting key input handlers
    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            self.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_S:
            self.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_D:
            self.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_W:
            self.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_E:
            self.cmd_update(0, 0, 1)
        elif key == Qt.Key.Key_Q:
            self.cmd_update(0, 0, -1)
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            self.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_S:
            self.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_D:
            self.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_W:
            self.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_E:
            self.cmd_update(0, 0, -1)
        elif key == Qt.Key.Key_Q:
            self.cmd_update(0, 0, 1)
        event.accept()

    def closeEvent(self, event):
        self.cleanup()

    #Methods for quickly changing style sheets
    def greenify(self, item):
        item.setStyleSheet("background-color: rgb(135, 255, 135); color: rgb(0, 0, 0);")

    def redify(self, item):
        item.setStyleSheet("background-color: rgb(255, 155, 155); color: rgb(0, 0, 0);")

    def plainify(self, item):
        item.setStyleSheet("")

    def plainify_modes(self):
        self.plainify(self.ui.node_control)
        self.plainify(self.ui.crawling)
        self.plainify(self.ui.calibration)

    #Method for quickly logging to the faux terminal
    def term_log(self, text):
        time = datetime.now().strftime("%m/%d/%y %H:%M:%S")
        self.ui.term.appendPlainText(f"{time} - {text}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
