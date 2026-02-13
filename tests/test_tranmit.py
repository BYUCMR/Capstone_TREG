import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))
import numpy as np
import serial
import rift.rover as rover
from rift.robot import RobotInverse
import rift.constrain as cstr
from rift.transmit.conversion import *

SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 9600


if __name__ == "__main__":
    config = rover.setup_rover_builder(1)
    robot = rover.make_robot(config)
    t = 1.5 # seconds
    payload_mass = np.zeros(12)
    payload_mass[rover.PAYLOAD] = 1
    motion = cstr.CompoundConstraint([cstr.Motion.make(rover.CR1,z=.1),
                                      cstr.Motion.lock(rover.CR2),
                                      cstr.Motion.lock(rover.CL1),
                                      cstr.Motion.lock(rover.CL2),
                                      cstr.Motion.lock(rover.CPR3),
                                      cstr.Motion.lock(rover.CPL3),
                                      cstr.Motion.make(cstr.Point.com(payload_mass), x=0)
                                    ])
    try:
        ser = serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1)
        print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")

        time.sleep(2)

        for *_,d_roll in robot.take_step(motion,resolution=10,allow_redundant=True):
            # print(d_roll)
            cmnd = dist_to_ticks(18,d_roll)
            message = f"VEL:{','.join(str(int(c)) for c in cmnd.ravel())}\n"
            print(message)
            # send_stop(ser)
            # send_command(ser,cmnd)
            
            # user_input = input()

    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Close the serial port
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print(f"Serial port {SERIAL_PORT} closed.")

    



    
    