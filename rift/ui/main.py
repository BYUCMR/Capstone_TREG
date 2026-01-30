import sys, asyncio, pathlib
sys.path.append(str(pathlib.Path.cwd().parent.parent))
from datetime import datetime

from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
# from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6 import QtAsyncio

from ui_main import Ui_Control
from ui_vis import Ui_vis_window
from Handlers.joystick_handler import JoystickHandler

import rift.anim
from rift.robot import InverseKinematicsError, RobotInverse
from rift.arraytypes import Matrix
from rift.truss_config import ROVER_CONFIG as config

from rift.steps import Command

cmd_state = Command('offline',0,0,0)

class MainWindow(QMainWindow): #referenced as widget by sim window class
    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_Control()
        self.ui.setupUi(self)

        self.ui.term_log = self.term_log

        self.ui.term_log("Welcome to R.I.F.T. Control!")

        cmd_state.mode = 'offline'

        self.joystick_handler = JoystickHandler(self.ui)

        self.ui.selector_label.setVisible(False)
        self.ui.selector.setVisible(False)

        self.ui.sim_toggle.clicked.connect(self.toggle_sim)

        self.ui.forward.pressed.connect(lambda: sim_widget.cmd_update(1, 0, 0))
        self.ui.backward.pressed.connect(lambda: sim_widget.cmd_update(-1, 0, 0))
        self.ui.left.pressed.connect(lambda: sim_widget.cmd_update(0, -1, 0))
        self.ui.right.pressed.connect(lambda: sim_widget.cmd_update(0, 1, 0))
        self.ui.del_right.pressed.connect(lambda: sim_widget.cmd_update(0, 0, 1))
        self.ui.del_left.pressed.connect(lambda: sim_widget.cmd_update(0, 0, -1))

        self.ui.forward.released.connect(lambda: sim_widget.cmd_update(-1, 0, 0))
        self.ui.backward.released.connect(lambda: sim_widget.cmd_update(1, 0, 0))
        self.ui.left.released.connect(lambda: sim_widget.cmd_update(0, 1, 0))
        self.ui.right.released.connect(lambda: sim_widget.cmd_update(0, -1, 0))
        self.ui.del_right.released.connect(lambda: sim_widget.cmd_update(0, 0, -1))
        self.ui.del_left.released.connect(lambda: sim_widget.cmd_update(0, 0, 1))

        self.ui.crawling.clicked.connect(lambda: self.mode_select('crawling'))
        self.ui.node_control.clicked.connect(lambda: self.mode_select('node control'))
        self.ui.calibration.clicked.connect(lambda: self.mode_select('calibration'))

    def toggle_sim(self):
        self.greenify(self.ui.sim_label)
        self.ui.sim_label.setText("Simulation Online")
        self.ui.sim_toggle.setText("Open Simulation")
        self.ui.term_log("Simulation Initialized")
        sim_widget.startup()

    def mode_select(self, mode):
        if mode == "crawling":
            self.plainify_modes()
            self.greenify(self.ui.crawling)
            cmd_state.mode = 'crawling'
            self.ui.selector_label.setVisible(False)
            self.ui.selector.setVisible(False)
        elif mode == "node_control":
            self.plainify_modes()
            self.greenify(self.ui.node_control)
            cmd_state.mode = 'node control'
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Node")
            self.ui.selector.setVisible(True)
        elif mode == "calibration":
            self.plainify_modes()
            self.greenify(self.ui.calibration)
            # cmd_state.mode = 'calibration'
            self.ui.selector_label.setVisible(True)
            self.ui.selector_label.setText("Roller")
            self.ui.selector.setVisible(True)
        sim_widget.control_mode = mode
        self.ui.term_log(f"Control Mode switched to {mode}")

    #overwriting key input handlers
    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            sim_widget.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_S:
            sim_widget.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_D:
            sim_widget.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_W:
            sim_widget.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_E:
            sim_widget.cmd_update(0, 0, 1)
        elif key == Qt.Key.Key_Q:
            sim_widget.cmd_update(0, 0, -1)
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key.Key_A:
            sim_widget.cmd_update(0, 1, 0)
        elif key == Qt.Key.Key_S:
            sim_widget.cmd_update(1, 0, 0)
        elif key == Qt.Key.Key_D:
            sim_widget.cmd_update(0, -1, 0)
        elif key == Qt.Key.Key_W:
            sim_widget.cmd_update(-1, 0, 0)
        elif key == Qt.Key.Key_E:
            sim_widget.cmd_update(0, 0, -1)
        elif key == Qt.Key.Key_Q:
            sim_widget.cmd_update(0, 0, 1)
        event.accept()

    #Methods for quickly changing style sheets
    def greenify(self, item):
        item.setStyleSheet("background-color: rgb(135, 255, 135); color: rgb(0, 0, 0);")

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


class SimWindow(QWidget): #referenced as sim_widget by mainwindow class
    forward = Signal()
    backward = Signal()
    right = Signal()
    left = Signal()
    del_left = Signal()
    del_right = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_vis_window()
        self.ui.setupUi(self)

        self.task_running = False
        self.view_live = False
        self.control_mode = "offline"

    def startup(self):
        if not self.view_live:
            self.animator = rift.anim.Animator.from_config(config)
            self.animator.view.setFocusPolicy(Qt.NoFocus)
            self.robot = RobotInverse.from_config(config)
            self.positions = asyncio.Queue[Matrix](50)
            self.ui.layout.addWidget(self.animator.view)
            self.loop = asyncio.get_event_loop()
            self.view_live = True
        self.show()

    def cmd_update(self, x: float, y: float, z: float):
        cmd_state.x += x
        cmd_state.y += y
        cmd_state.z += z
        print(f"X: {cmd_state.x}, Y: {cmd_state.y}, Z: {cmd_state.z}")

    #Example run functions
    async def crawl(self) -> None: #will be obsolete once we plumb in real crawling code
        for _ in self.robot.crawl(1, 0.8, resolution=50):
            await self.positions.put(self.robot.pos.copy())

    async def crawl_execution(self): #will be obsolete once we plumb in real crawling code
        self.task_running = True
        crawling_task = asyncio.create_task(self.crawl())
        animation_task = asyncio.create_task(self.animator.animate(self.positions))
        try:
            await crawling_task
        except InverseKinematicsError as e:
            print(e.args[0])
        self.task_running = False
        await animation_task

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    sim_widget = SimWindow()
    widget = MainWindow()
    widget.show()
    QtAsyncio.run()
