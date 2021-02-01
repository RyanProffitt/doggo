# All-Purpose Robot
# February 1 2021
# Ryan Proffitt
# This module contains all of the functionality for generating Doggo telecommands.

#TODO: Put some of this in C

from enum import Enum
import time

#----------------General-------------------#

# All possible command types
class CmdType(Enum):
    CMD_NOP = 0x00
    CMD_STS = 0x01
    CMD_MOTOR_CTRL = 0x02

# All possible telemetry types
class TlmType(Enum):
    TLM_ALIVE = 0x00
    TLM_STS = 0x01
    TLM_MOTOR_CTRL = 0x02

TLM_START_BYTE_VAL = 0x50
TLM_MACHINE_ID = 0x44
TLM_END_BYTE_VAL = 0x51

# Manages Tctm
# Maintains the serial connection, sends commands, and handles telemetry
class TctmManager(ser_connection):
    def __init__(self, serial_connection):
        self.my_serial_conn = serial_connection

    def send_cmd(cmd):
        isinstance(cmd, Telecommand)
        my_serial_conn.write(cmd._to_bytes)

    def recv_tlm():
        byte_read = b'0x01'

        while(byte_read):
            # Check for Start Byte #1
            byte_read = ser.read(1) # Leave this here for the while loop to check
            if int.from_bytes(byte_read, "big") != TLM_START_BYTE_VAL:
                continue

            # Start Byte #2
            if int.from_bytes(ser.read(1), "big") != TLM_START_BYTE_VAL:
                continue

            # The Machine ID
            machine_id = int.from_bytes(ser.read(1), "big")
            if machine_id != TLM_MACHINE_ID:
                continue

            # The Telemetry Type
            tlm_type = int.from_bytes(ser.read(1), "big")
            if not tlm_type in set(TlmType):
                continue

            # At this point we have valid telemetry from a machine, let's collect the rest
            recv_time = time.time()
            tlm_count = int.from_bytes(ser.read(1), "big")
            tlm_ms_since_boot = int.from_bytes(ser.read(4), "big")
            tlm_data_len = int.from_bytes(ser.read(1), "big")

            tlm_data = list(ser.read(tlm_data_len))

            if != TLM_END_BYTE_VAL and AA != TLM_END_BYTE_VAL:
                

            if(tlm[-1] == TLM_END_BYTE_VAL and tlm[-2] == TLM_END_BYTE_VAL):
                state.time_hk_received = tmp_time
                parse_tlm(tlm, state)
                #print(state)
                #print(binascii.hexlify(raw_tlm))
                tlm_file.write(str(tlm), "\n")

#-----------------Telecommands------------------#

# The basic structure of a telecommand
# The telecommand bytes are capped by Start and End bytes
# cmd_type - The command type of type Enum CmdType
# cmd_data - A list of byte values that mark the command data
#       ! It is up to the telecommand generator to pack these bytes correctly !
class Telecommand():
    def __init__(self, cmd_type, cmd_data):
        self.start_byte0 = 0x61
        self.start_byte1 = 0x61
        self.cmd_type = cmd_type
        self.cmd_data = cmd_data
        self.end_byte0 = 0x62
        self.end_byte0 = 0x62

    def _to_bytes():
        cmd_bytes = [self.start_byte0, self.start_byte1, self.cmd_type]
        cmd_bytes.extend(cmd_type)
        cmd_bytes.extend([self.end_byte0, self.end_byte1])
        return bytes(cmd_bytes)

# Generates command
# Accepts two tuples
#   leftMotor - (pwm_val, motor_action)
#   rightMotor - (pwm_val, motor_action)
#       pwm_val - A byte representing the PWM value that controls motor power
#       motor_action - An Enum MotorAction representing the motor direction
def gen_cmd_motor_ctrl(leftMotor, rightMotor):
    if not (0 <= leftMotor[0] <= 255 and 0 <= rightMotor[0] <= 255):
        raise ValueError("pwm_val must be uint8")
    if not (isinstance(leftMotor[1], MotorAction) and isinstance(rightMotor[1], MotorAction)):
        raise ValueError("motor_action must be of type MotorAction")

    return Telecommand(CMD_MOTOR_CTRL, [leftMotor[0], leftMotor[1], int(rightMotor[0]), int(rightMotor[1]]))

#----------------Telemetry-------------------#
class Telemetry():
    def __init__(self, read_time, tlm_type, tlm_data):
        self.read_time = read_time
        self.tlm_type = tlm_type
        self.tlm_data = tlm_data

def test_tctm():
    #TODO
    print("Blah")