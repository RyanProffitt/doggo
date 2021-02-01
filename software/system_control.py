#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This is the core logic of the control module (Raspberry Pi 4B).

import serial
import binascii
import time
from enum import Enum

class MotorAction(Enum):
    BACKWARD = 0
    NEUTRAL = 1
    FORWARD = 2

class State:
    def __init__(self):
        machine_id = 0
        tlm_cnt = 0
        motor0_speed = 0
        motor1_speed = 0
        
        tlm = []
        time_hk_received = 0
        time_hk_reported_sent = 0
        
    def __str__(self):
        return "State:: MachineID:{0}, TlmCnt: {1}, Motor1:{2}, Motor0:{3}, HK_SentTime:{4}".format(self.machine_id, self.tlm_cnt, self.motor0_speed,
            self.motor1_speed, str(round(self.time_hk_reported_sent / 1000, 2)) + " sec")
        
#Parses telemetry and updates the State
#Accepts telemetry as a list of ints and a State object
def parse_tlm(tlm, state):
    state.tlm = tlm
    state.machine_id = tlm[2]
    state.tlm_cnt = tlm[3]
    state.motor0_speed = tlm[4]
    state.motor1_speed = tlm[5]
    state.time_hk_reported_sent = int.from_bytes(bytes(tlm[6:10]), "big")

def listen_for_hk(tctm_manager, state, tlm_file):
    tctm_manager.recv_tlm()

def main():
    tlm_file = open("/home/pi/doggo/software/tlm_files/tlm_output") #TODO: time based file names

    print("Setting up serial connection...", end=" ")
    try:
        serial_conn = serial.Serial("/dev/ttyACM0")
    except(Exception as e):
        print(e, "\nExiting gracefully.")
        close(tlm_file)
        exit()
    print("done!")

    
    state = State()
    listen_for_hk(serial_conn, state, tlm_file)
    command
    

    print("Listening ")
    
if __name__ == "__main__":
    main()