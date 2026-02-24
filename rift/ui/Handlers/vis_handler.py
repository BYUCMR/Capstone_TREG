from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot
from PySide6.QtWidgets import QWidget
from Handlers.ui_vis import Ui_vis_window
import time
from numpy import ndarray

from rift.steps import Mode
from rift import rover
from rift.robot import InverseKinematicsError
from rift.arraytypes import Matrix
from rift.rover import make_animator, make_robot

class SimWindow(QWidget): #referenced as sim_widget by mainwindow class

    send_cmd = Signal(Mode, int, float, float, float)
    send_startup = Signal()

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


        self.send_startup.connect(self.worker.setup)
        self.send_startup.emit()
        self.send_cmd.connect(self.worker.run_next)
        self.worker.ready.connect(self.send_new)
        self.worker.anim_update.connect(self.update_anim)
        self.thread.finished.connect(self.worker.deleteLater)
        self.send_new()
        self.thread.start()

    def update_anim(self, matrix):
        print("update matrix yo")

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
        animator = make_animator()
        self.ui.layout.addWidget(animator.view)
        self.view_live = True

    def kill_sim(self):
        print('nuh')
        # self.thread.requestInterruption()
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
    anim_update = Signal(ndarray)
    message = Signal(str)

    @Slot()
    def setup(self):
        self.robot = make_robot()

    @Slot(Mode, int, float, float, float)
    def run_next(self, mode, item, x, y, z):
        cur_thread = QThread.currentThread()
        if(cur_thread.isInterruptionRequested()):
            cur_thread.exit()
            return
        #do other stuff with this very helpful data!

        time.sleep(1)
        self.ready.emit()



