import numpy as np
import serial
import time

def dist_to_ticks(perimeter, command):
    ticks_per_foot = 1125*12
    cmd_np = np.array(command)*perimeter*ticks_per_foot
    return cmd_np

def ticks_to_tps(command, t=1.5):
    return command/t

def send_command(ser, commands,t):
    commands = commands
    
    message = f"VEL_DUR:{','.join(str(int(c)) for c in commands.ravel())}"+f":{t}\n"
    ser.write(message.encode('utf-8'))
    ser.flush()
    time.sleep(2) # can adjust, 1.5 matches transmitter
    print(f"[SENT] {message.strip()}")

def send_command_pos(ser, commands,t):
    # commands = -1* commands
    
    message = f"Pos:{','.join(str(int(c)) for c in commands.ravel())}\n"
    ser.write(message.encode('utf-8'))
    ser.flush()
    time.sleep(2) # can adjust, 1.5 matches transmitter
    print(f"[SENT] {message.strip()}")
    return message

def send_stop(ser):
    message = "STOP\n"
    ser.write(message.encode('utf-8'))
    ser.flush()
    print(f"[SENT] {message.strip()}")

def read_positions(ser):
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if line.startswith('[') and line.endswith(']'):
            try:
                values = np.fromstring(line[1:-1], sep=',')
                return -values.reshape(-1, 1)
            except Exception:
                print(f"[WARN] Could not parse: {line}")
                return None
        else:
            # Print other serial messages like warnings
            print(f"[OTHER] {line}")
            return