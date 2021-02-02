#All-Purpose Robot
#1 Feb 2021
#Ryan Proffitt

# This is the core logic of the control module (Raspberry Pi 4B).

import serial
import time
from multiprocessing import Process
import keyboard

import machine_state
import tctm
from tctm import MotorAction as MotorAction
from tctm import gen_cmd_motor_ctrl

def listen_for_tlm(tctm_manager, state, tlm_file):
    while(True):
        tlm, error = tctm_manager.recv_tlm()

        if tlm:
            tlm_file.write(str(tlm) + "\n")
        elif error:
            tlm_file.write(str(error) + "\n")

def main():
    robo_state = machine_state.MachineState(0x44, serial_port="/dev/ttyACM0")

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
    tlm_process = Process(target=listen_for_tlm, args=(tctm_manager, robo_state, tlm_file,))
    tlm_process.start()

    tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (175, MotorAction.FORWARD), (175, MotorAction.FORWARD)))

    time.sleep(5)

    # Keyboard Control of Bot
    # while True:
    #     try:
    #         if keyboard.is_pressed("w"):
    #             tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (255, MotorAction.FORWARD), (255, MotorAction.FORWARD)))
    #         elif keyboard.is_pressed("s"):
    #             tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (255, MotorAction.BACKWARD), (255, MotorAction.BACKWARD)))
    #         elif keyboard.is_pressed("a"):
    #             tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (200, MotorAction.BACKWARD), (200, MotorAction.FORWARD)))
    #         elif keyboard.is_pressed("d"):
    #             tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (200, MotorAction.FORWARD), (200, MotorAction.BACKWARD)))
    #         elif keyboard.is_pressed("q"):
    #             break
    #         else:
    #             tctm_manager.send_cmd(gen_cmd_motor_ctrl(robo_state.machine_id, (0, MotorAction.NEUTRAL), (0, MotorAction.NEUTRAL)))
    #     except Exception as e:
    #         print(e)

    #     time.sleep(0.010)
    
    tlm_file.close()
    
if __name__ == "__main__":
    main()