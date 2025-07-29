import serial.tools.list_ports

def find_serial_ports_by_pid_vid():
    """
    Scanează porturile seriale și returnează un dict cu porturile pentru robot și motoare
    folosind PID și VID definite.
    """
    # PID și VID din codul tău
    PID_ROBOT = 29987
    VID_ROBOT = 6790

    PID_MOTORS = 67
    VID_MOTORS = 10755

    ports = {
        'robot': None,
        'motors': None
    }

    for port in serial.tools.list_ports.comports():
        # port.vid și port.pid pot fi None, deci verificăm întâi asta
        if port.vid is not None and port.pid is not None:
            if port.pid == PID_ROBOT and port.vid == VID_ROBOT:
                ports['robot'] = port.device
            elif port.pid == PID_MOTORS and port.vid == VID_MOTORS:
                ports['motors'] = port.device

    return ports

# Exemplu de folosire:
ports = find_serial_ports_by_pid_vid()
print(f"Port robot: {ports['robot']}")
print(f"Port motors: {ports['motors']}")
