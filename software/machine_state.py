#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This provides functionality for monitoring the machine state.

from datetime import datetime
import serial

import tctm

class MachineState:
    def __init__(self, machine_id, ):
        self._machine_id = machine_id

        # The history of commands that were SENT to machine
        self._cmd_history = [] #TODO

        self.tctm_manager = tctm.TctmManager()

    def quick_report(self):
        return "State ({0}), ID: {1}".format(datetime.now().strftime("%H:%M:%S"), self._machine_id)
