# RIFT User Interface
This folder contains code for the User Interface built to help with simulation and control of the RIFT rover. The UI is built using the PySide6 package, a Python implementation of the Qt Framework.

## main.py
This is the runnable file for the UI, built around the `MainWindow` class. If you want to use the UI, this is the only file you will need to run.

The UI's execution of simulation, animation, and control are done using separate threads (using PySide6's `QThread` class). The actual User Interface runs on a main thread, and separate thread instances are opened for simulations and joystick inputs, as needed.

## Handlers/vis_handler.py
This file contains the class for simulation and visualization. The `SimWindow` class is implemented into the main UI thread, and the `VizWorker` class is implemented on a separate thread for more resource intensive tasks as needed.

## Handlers/joystick_handler.py
This file contains the `JoystickHandler` class to set up joystick control using the Pygame package, and the `JoyWorker` class to take live inputs. Because Pygame uses a while loop structure, the `JoyWorker` class is implemented on a separate thread.
