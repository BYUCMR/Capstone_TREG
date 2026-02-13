import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))
import numpy as np
import serial
from rift.truss_config import setup_rover_builder
from rift.steps import make_step_array
import rift.indices as I
from rift.transmit import step_in
from rift.robot import RobotInverse
from rift.transmit.conversion import *

SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 9600


if __name__ == "__main__":
    config = setup_rover_builder(1)
    robot = RobotInverse.from_config(config)
    t = 1.5 # seconds
    steps = make_step_array(
        (12,3),
        (I.R2,0),
        (I.L2,0),
        (I.R1,0),
        (I.L1,np.array([-.1,np.nan,np.nan])),
    )
    try:
        ser = serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1)
        print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")

        time.sleep(2)

        for *_,d_roll in robot.take_step(steps):
            # print(d_roll)
            cmnd = dist_to_ticks(18,d_roll)
            message = f"VEL:{','.join(str(int(c)) for c in cmnd.ravel())}\n"
            print(message)
            send_command(ser,cmnd)
            send_stop(ser)
            user_input = input()

    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Close the serial port
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print(f"Serial port {SERIAL_PORT} closed.")

    



    
    