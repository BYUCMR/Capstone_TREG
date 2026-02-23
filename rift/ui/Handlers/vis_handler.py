from PySide6.QtCore import Qt, QObject, Signal
from PySide6.QtWidgets import QWidget
from Handlers.ui_vis import Ui_vis_window

from rift import rover
from rift.robot import InverseKinematicsError
from rift.arraytypes import Matrix

class SimWindow(QWidget): #referenced as sim_widget by mainwindow class

    def __init__(self, cmd_update, parent=None):
        super().__init__(parent)

        self.ui = Ui_vis_window()
        self.ui.setupUi(self)

        self.cmd_update = cmd_update

        self.task_running = False
        self.view_live = False

    def start_sim(self):
        self.show()
        print('yuh')
        self.show()
        self.view_live = True

    def kill_sim(self):
        print('nuh')
        self.view_live = False

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


class VizWorker(QObject):
    ready = Signal()

