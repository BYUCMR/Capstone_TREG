from os import environ
environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import warnings
warnings.filterwarnings("ignore")
import pygame
from PySide6.QtCore import QObject, QThread, Signal

class JoystickHandler:

    def __init__(self, ui):
        self.ui = ui
        self.ui.js_toggle.clicked.connect(self.start_joystick)

    def start_joystick(self):
        self.js_worker = JoyWorker()
        self.js_thread = QThread()

        self.js_worker.moveToThread(self.js_thread)

        self.js_thread.started.connect(self.js_worker.heavy_task)
        self.js_worker.finished.connect(self.js_thread.quit)
        self.js_worker.finished.connect(self.js_worker.deleteLater)
        self.js_thread.finished.connect(self.js_thread.deleteLater)

        self.js_worker.message.connect(self.ui.term_log)

        self.js_thread.start()


class JoyWorker(QObject):
    action = Signal()
    message = Signal(str)
    finished = Signal()

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
                if QThread.currentThread().isInterruptionRequested:
                    self.hosting = False
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        self.message.emit("Joystick Button Pressed")
                        self.action.emit()
            self.finished.emit()
                # time.sleep(0.5)
