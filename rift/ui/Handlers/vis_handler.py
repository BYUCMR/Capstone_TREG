import numpy as np
from numpy import ndarray
from PySide6.QtCore import Qt, QObject, Signal, QThread, Slot
from PySide6.QtWidgets import QApplication

from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.tubetruss.robots import InverseKinematicsError, TrussRobot
from .controls import Bundler, Command, take_command


class SimWindow(QObject): #referenced as sim_widget by mainwindow class
    send_cmd = Signal(Command)
    message = Signal(str)
    reset = Signal(ndarray)

    work_thread: QThread | None = None

    def __init__(
        self,
        cmd_state: Command,
        parent: QObject | None = None
    ) -> None:
        super().__init__(parent)
        self.cmd_state = cmd_state
        self.view, self.animate = rover.set_up_animation(
            rover.ROLLING_POS,
            trace_len=10,
        )
        self.view.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.worker = VizWorker(period=10)
        self.send_cmd.connect(self.worker.run_cmd)
        self.worker.done.connect(self.send_new)
        self.worker.results.connect(self.update_anim)
        self.worker.message.connect(self.message.emit)
        self.reset.connect(self.worker.reset)
        self.reset.connect(self.update_anim)

    @property
    def sim_live(self) -> bool:
        return self.work_thread is not None and self.work_thread.isRunning()

    @Slot(ndarray)
    @Slot(ndarray, ndarray)
    def update_anim(self, x: Matrix, dq: Vector | None = None) -> None:
        self.animate(x)

    @Slot()
    def send_new(self):
        self.send_cmd.emit(self.cmd_state)

    def start_sim(self) -> None:
        self.work_thread = QThread()
        self.worker.moveToThread(self.work_thread)
        self.work_thread.finished.connect(self.worker.pull_to_main)
        self.send_new()
        self.work_thread.start()

    def kill_sim(self) -> None:
        self.reset.emit(rover.ROLLING_POS)
        if self.work_thread is not None:
            self.work_thread.requestInterruption()


class VizWorker(QObject):
    done = Signal()
    results = Signal(ndarray, ndarray)
    message = Signal(str)

    def __init__(
        self,
        init_pos: Matrix = rover.ROLLING_POS,
        *,
        period: int = 1,
        stabilize: bool = False,
    ) -> None:
        super().__init__()
        self.robot = rover.make_robot(init_pos)
        if stabilize:
            self.stabilizer = rover.make_stabilizer(init_pos)
        else:
            self.stabilizer = None
        self.bundler = Bundler(period)
        self.gen = None

    @Slot(ndarray)
    def reset(self, pos: Matrix) -> None:
        self.bundler.delta_q = None
        self.robot = TrussRobot(
            pos.copy(),
            self.robot.truss,
            self.robot.control,
        )
        if self.stabilizer is not None:
            self.stabilizer = rover.make_stabilizer(pos)

    @Slot()
    def run_next(self) -> None:
        cur_thread = QThread.currentThread()
        if cur_thread.isInterruptionRequested():
            cur_thread.exit()
            return
        if self.gen is None:
            return
        try:
            delta_q = next(self.gen)
        except StopIteration:
            self.gen = None
            self.done.emit()
        except InverseKinematicsError as e:
            self.gen = None
            self.done.emit()
            self.message.emit(e.args[0])
        else:
            if self.stabilizer is not None:
                self.stabilizer.update_pos(self.robot.pos)
                out_pos = self.stabilizer.pos
            else:
                out_pos = self.robot.pos.copy()
            self.results.emit(out_pos, delta_q)

    @Slot(Command)
    def run_cmd(self, cmd: Command) -> None:
        cur_thread = QThread.currentThread()
        if cur_thread.isInterruptionRequested():
            cur_thread.exit()
            return
        gen = take_command(self.robot, cmd)
        self.gen = self.bundler.expend(gen)
        self.run_next()

    @Slot()
    def pull_to_main(self) -> None:
        app = QApplication.instance()
        if app is not None:
            self.moveToThread(app.thread())
