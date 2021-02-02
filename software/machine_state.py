#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This provides functionality for monitoring the machine state.

from datetime import datetime

class MachineState:
    def __init__(self, machine_id, serial_port=""):
        self.serial_port = serial_port

        self.machine_id = machine_id
        self._is_alive = True

        self.recvd_cmds = []

    def quick_report(self):
        report_time = datetime.now().strftime("%H:%M:%S")
        return "State ({0}), ID: {1}, Alive: {2}, Last Cmd: {3}".format(report_time, self.machine_id, self._is_alive, self.recvd_cmds[-1])
