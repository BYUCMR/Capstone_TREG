from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot
from PySide6.QtWidgets import QWidget
from Handlers.ui_vis import Ui_vis_window
import time

from rift.steps import Mode
from rift import rover
from rift.robot import InverseKinematicsError
from rift.arraytypes import Matrix

class SimWindow(QWidget): #referenced as sim_widget by mainwindow class

    send_cmd = Signal(Mode, int, float, float, float)

    def __init__(self, cmd_state, parent=None):
        super().__init__(parent)

        self.ui = Ui_vis_window()
        self.ui.setupUi(self)

        self.cmd_state = cmd_state

        self.task_running = False
        self.view_live = False

        self.thread = QThread()
        self.worker = VizWorker()
        self.worker.moveToThread(self.thread)

        self.send_cmd.connect(self.worker.run_next)
        self.worker.ready.connect(self.send_new)
        self.worker.message.connect(print)
        self.thread.finished.connect(self.worker.deleteLater)
        # print('thread starting')
        self.send_new()
        self.thread.start()


    def send_new(self):
        mode = self.cmd_state.mode
        item = self.cmd_state.item
        x = self.cmd_state.x
        y = self.cmd_state.y
        z = self.cmd_state.z
        self.send_cmd.emit(mode, item, x, y, z)

    def start_sim(self):
        self.show()
        print('yuh')
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
    message = Signal(str)

    @Slot(Mode, int, float, float, float)
    def run_next(self, mode, item, x, y, z):
        cur_thread = QThread.currentThread()
        if(cur_thread.isInterruptionRequested()):
            cur_thread.exit()
            return
        self.message.emit(str(x))
        #do other stuff with this very helpful data!

        time.sleep(1)
        self.ready.emit()



