import time

from numpy import ndarray
from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot
from PySide6.QtWidgets import QWidget

from rift import rover
from rift.arraytypes import Matrix
from rift.steps import Command
from .ui_vis import Ui_vis_window

class SimWindow(QWidget): #referenced as sim_widget by mainwindow class
    send_cmd = Signal(Command)
    send_startup = Signal()

    def __init__(
        self,
        cmd_state: Command,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self.ui = Ui_vis_window()
        self.ui.setupUi(self)

        self.cmd_state = cmd_state

        self.view_live = False

        view, self.animate = rover.set_up_animation()
        self.ui.layout.addWidget(view)
        view.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    @Slot(ndarray)
    def update_anim(self, matrix: Matrix) -> None:
        self.animate(matrix)

    @Slot()
    def send_new(self):
        self.send_cmd.emit(self.cmd_state)

    def start_sim(self) -> None:
        self.show()
        # print('yuh')

        self.work_thread = QThread()
        self.worker = VizWorker()
        self.worker.moveToThread(self.work_thread)

        self.send_startup.connect(self.worker.setup)
        self.send_startup.emit()
        self.send_cmd.connect(self.worker.run_next)
        self.worker.ready.connect(self.send_new)
        self.worker.anim_update.connect(self.update_anim)
        self.work_thread.finished.connect(self.worker.deleteLater)
        self.send_new()
        self.work_thread.start()

        self.view_live = True

    def kill_sim(self) -> None:
        # print('nuh')
        self.hide()
        self.work_thread.requestInterruption()
        self.view_live = False


class VizWorker(QObject):
    ready = Signal()
    anim_update = Signal(ndarray)
    message = Signal(str)

    @Slot()
    def setup(self) -> None:
        self.robot = rover.make_robot()

    @Slot(Command)
    def run_next(self, cmd: Command) -> None:
        cur_thread = QThread.currentThread()
        if cur_thread.isInterruptionRequested():
            cur_thread.exit()
            return
        for _ in rover.take_command(self.robot, cmd, resolution=100):
            self.anim_update.emit(self.robot.pos.copy())
            time.sleep(0.01)
        self.ready.emit()
