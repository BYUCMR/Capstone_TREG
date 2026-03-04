import numpy as np
from numpy import ndarray
from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot

from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.robot import InverseKinematicsError
from rift.steps import Command


class SimWindow(QObject): #referenced as sim_widget by mainwindow class
    send_cmd = Signal(Command)
    message = Signal(str)

    def __init__(
        self,
        cmd_state: Command,
        ui,
        parent: QObject | None = None
    ) -> None:
        super().__init__(parent)
        self.cmd_state = cmd_state
        self.sim_live = False
        view, self.animate = rover.set_up_animation(trace_len=10)
        ui.ctr_layout.insertWidget(0, view)
        view.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    @Slot(ndarray, ndarray)
    def update_anim(self, x: Matrix, dq: Vector) -> None:
        self.animate(x)

    @Slot()
    def send_new(self):
        self.send_cmd.emit(self.cmd_state)

    def start_sim(self) -> None:
        self.work_thread = QThread()
        self.worker = VizWorker(period=10)
        self.worker.moveToThread(self.work_thread)

        self.send_cmd.connect(self.worker.run_cmd)
        self.worker.done.connect(self.send_new)
        self.worker.results.connect(self.update_anim, Qt.ConnectionType.BlockingQueuedConnection)
        self.worker.message.connect(self.message.emit)
        self.work_thread.finished.connect(self.worker.deleteLater)
        self.send_new()
        self.work_thread.start()

        self.sim_live = True

    def kill_sim(self) -> None:
        self.work_thread.requestInterruption()
        self.sim_live = False


class VizWorker(QObject):
    done = Signal()
    results = Signal(ndarray, ndarray)
    message = Signal(str)

    def __init__(
        self,
        init_pos: Matrix = rover.CRAWLING_POS,
        *,
        resolution: int = 100,
        period: int = 1,
    ) -> None:
        super().__init__()
        self.period = period
        self.resolution = resolution
        self.robot = rover.make_robot(init_pos)

    @Slot(Command)
    def run_cmd(self, cmd: Command) -> None:
        cur_thread = QThread.currentThread()
        if cur_thread.isInterruptionRequested():
            cur_thread.exit()
            return
        gen = rover.take_command(self.robot, cmd, resolution=self.resolution)
        delta_q = np.zeros(self.robot.n_rollers)
        try:
            for i, dq in enumerate(gen):
                delta_q += dq
                if not i % self.period:
                    self.results.emit(self.robot.pos.copy(), delta_q)
        except InverseKinematicsError as e:
            self.message.emit(e.args[0])
        self.done.emit()
