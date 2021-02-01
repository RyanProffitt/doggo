#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This provides functionality for monitoring the machine state.

from datetime import datetime

class MachineState:
    def __init__(self, serial_port=""):
        self.serial_port = serial_port

        self.machine_id = 0

        self.recvd_cmds = []

    def quick_report(self):
        report_time = datetime.now().strftime("%H:%M:%S")
        return "State ({0}), ID: {1}, Alive: {2}, Last Cmd: {3}".format(report_time, self.machine_id, self.recvd_cmds[-1])
