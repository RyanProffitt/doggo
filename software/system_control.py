#Doggo
#29 Apr 2020
#Ryan Proffitt

#This code is the main "brain" of Doggo.

import serial
import binascii
import time

class State:
    def __self__():
        motor0_speed = 0.
        motor1_speed = 0.
        
        tlm = []
        time_hk_received = 0
        time_hk_reported_sent = 0
        
    def __str__(self):
        return "State[Motor1:{0}, Motor0:{1}, HK_Sent:{2}]".format(self.motor0_speed,
            self.motor1_speed, str(round(self.time_hk_reported_sent / 1000, 2)) + " sec")
        
#Parses telemetry and updates the State
#Accepts telemetry as a list of ints and a State object
def parse_tlm(tlm, state):
    state.tlm = tlm
    state.motor0_speed = tlm[2]
    state.motor1_speed = tlm[3]
    state.time_hk_reported_sent = int.from_bytes(bytes(tlm[4:8]), "big")

def listen_for_hk(ser, state):
    byte_ctr = 0
    
    print("Listening for telemetry.")

    while(True):
        byte_read = ser.read(1)
        if(int.from_bytes(byte_read, "big") == 0x50):
            tmp_time = time.time()
            raw_tlm = byte_read
            byte_read = ser.read(1)
            if(int.from_bytes(byte_read, "big") == 0x50):
                raw_tlm = raw_tlm + byte_read + ser.read(8)
                tlm = list(raw_tlm)
                if(tlm[-1] == 0x51 and tlm[-2] == 0x51):
                    state.time_hk_received = tmp_time
                    parse_tlm(tlm, state)
                    print(state)
                    #print(binascii.hexlify(raw_tlm))

def main():
    state = State()
    ser = serial.Serial("/dev/ttyACM0")
    listen_for_hk(ser, state)
    
if __name__ == "__main__":
    main()