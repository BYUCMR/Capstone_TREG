from os import environ
import warnings
import time

environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
warnings.filterwarnings("ignore")

import pygame
from PySide6.QtCore import QObject, QThread, Signal, Slot


class JoystickHandler(QObject):
    message = Signal(str)
    command = Signal(float,float,float)

    def __init__(self, ui,parent: QObject | None = None):
        super().__init__(parent)
        self.ui = ui
        self.message.emit("test")
        self.ui.js_toggle.clicked.connect(self.start_joystick)

    @Slot()
    def start_joystick(self):
        self.js_worker = JoyWorker()
        self.js_thread = QThread()

        self.js_worker.moveToThread(self.js_thread)

        self.js_thread.started.connect(self.js_worker.heavy_task)
        self.js_worker.finished.connect(self.js_thread.quit)
        self.js_worker.finished.connect(self.js_worker.deleteLater)
        self.js_thread.finished.connect(self.js_thread.deleteLater)

        self.js_worker.message.connect(self.message.emit)
        self.js_worker.action.connect(self.command.emit)

        self.js_thread.start()

    def message_pr(self,msg):
        print(msg)


class JoyWorker(QObject):
    action = Signal(float, float, float)
    message = Signal(str)
    finished = Signal()

    @Slot()
    def heavy_task(self):
        self.message.emit("Pygame Running")
        pygame.init()
        pygame.joystick.init()
        if(pygame.joystick.get_count() == 0):
            pygame.joystick.quit()
            pygame.quit()
            self.finished.emit()
            self.message.emit("No Joystick Found")
        else:
            joystick = pygame.joystick.Joystick(0)
            self.message.emit(f"Jostick Initialized: {joystick.get_name()}")
            self.hosting = True
            while self.hosting:
                # for event in pygame.event.get():
                #     if event.type == pygame.JOYBUTTONDOWN:
                #         self.message.emit("Joystick Button Pressed")
                #         self.action.emit()
                pygame.event.pump()
                x = joystick.get_axis(0)
                if x < 0.1 and x > -0.1: x = 0
                y = joystick.get_axis(1)
                if y < 0.1 and y > -0.1: y = 0
                z = joystick.get_axis(2)
                if z < 0.1 and z > -0.1: z = 0
                self.action.emit(x,y,z)
                time.sleep(0.1)
            self.finished.emit()
                # time.sleep(0.5)
