#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This is the core logic of the control module (Raspberry Pi 4B).

import serial
import multiprocessing

import machine_state
import tctm

def listen_for_hk(tctm_manager, state, tlm_file):
    while(True):
        tlm, error = tctm_manager.recv_tlm()

        if tlm:
            print(tlm)
        elif error:
            print(error)

def main():
    robo_state = machine_state.MachineState(serial_port="/dev/ttyACM0")

    print("Setting up serial connection with machine...", end=" ")
    try:
        serial_conn = serial.Serial(robo_state.serial_port)
        tctm_manager = tctm.TctmManager(serial_conn)
        print("done!")
    except Exception as e:
        print("\n", e, "\nExiting gracefully.")
        exit()

    print("Initializing Machine State...", end=" ")
    #TODO: Initialize
    print("done!")

    print("Listening for telemetry and awaiting commands...")

    tlm_file = open("/home/pi/doggo/software/tlm_files/tlm_output", "w") #TODO: time based file names
    listen_for_hk(tctm_manager, robo_state, tlm_file)

    #TODO set up command issuing
    
    tlm_file.close()
    
if __name__ == "__main__":
    main()