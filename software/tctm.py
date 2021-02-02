# All-Purpose Robot
# February 1 2021
# Ryan Proffitt
# This module contains all of the functionality for generating Doggo telecommands.

#TODO: Put some of this in C

from enum import IntEnum
from datetime import datetime

#----------------General-------------------#

# All possible command types
class CmdType(IntEnum):
    CMD_NOP = 0x00
    CMD_STS = 0x01
    CMD_SYSTEM_CHECK = 0x02
    CMD_MOTOR_CTRL = 0x03
CmdType_VALUES = [c.value for c in set(CmdType)]

# All possible telemetry types
class TlmType(IntEnum):
    TLM_ALIVE = 0x00
    TLM_STS_RES = 0x01
    TLM_SYSTEM_CHECK_RES = 0x02
    TLM_MOTOR_CTRL_RES = 0x03
TlmType_VALUES = [t.value for t in set(TlmType)]

# General Constants in Component Module Code
MACHINE_ID = 0x44

CMD_START_BYTE_VAL = 0x60
CMD_END_BYTE_VAL = 0x61

TLM_START_BYTE_VAL = 0x50
TLM_END_BYTE_VAL = 0x51

class MotorAction(IntEnum):
    BACKWARD = 0x00,
    NEUTRAL = 0x01,
    FORWARD = 0x02

# Manages Tctm
# Maintains the serial connection, sends commands, and handles telemetry
class TctmManager():
    #TODO Make all command generators a function of this class
    def __init__(self, serial_connection=None):
        self.my_serial_conn = serial_connection

    def send_cmd(self, cmd):
        isinstance(cmd, Telecommand)
        try:
            self.my_serial_conn.write(cmd._to_bytes())
        except Exception as e:
            if self.my_serial_conn == None:
                raise ValueError("Serial connection was never initialized.")
            else:
                raise e

    # Collects telemetry from the serial connection and assembles Telemetry object
    # Returns:
    #   Telemetry Object, None if telemetry found
    #   None, Error if an error was encountered
    #   None, None if no telemetry was found and no error was encountered
    def recv_tlm(self):
        tlm = None
        byte_read = b'0x01'

        try:
            while(byte_read):
                # Check for Start Byte #1
                byte_read = self.my_serial_conn.read(1) # Leave this here for the while loop to check
                if int.from_bytes(byte_read, "big") != TLM_START_BYTE_VAL:
                    continue

                # Start Byte #2
                if int.from_bytes(self.my_serial_conn.read(1), "big") != TLM_START_BYTE_VAL:
                    continue

                # The Machine ID
                machine_id = int.from_bytes(self.my_serial_conn.read(1), "big")
                if machine_id != MACHINE_ID:
                    continue

                # The Telemetry Type
                tlm_type = int.from_bytes(self.my_serial_conn.read(1), "big")
                if not tlm_type in TlmType_VALUES:
                    continue

                # At this point we have valid telemetry from a machine, let's collect the rest
                recv_time = datetime.now()
                tlm_count = int.from_bytes(self.my_serial_conn.read(1), "big")
                cmd_count = int.from_bytes(self.my_serial_conn.read(1), "big")
                tlm_ms_since_boot = int.from_bytes(self.my_serial_conn.read(4), "big")
                tlm_data_len = int.from_bytes(self.my_serial_conn.read(1), "big")
                tlm_data = list(self.my_serial_conn.read(tlm_data_len))

                # Finally, Check the End Bytes
                if int.from_bytes(self.my_serial_conn.read(1), "big") != TLM_END_BYTE_VAL and \
                        int.from_bytes(self.my_serial_conn.read(1), "big")  != TLM_END_BYTE_VAL:
                    raise ValueError("Tlm Found, but End Bytes Missing...")

                # Construct Telemetry Object
                tlm = Telemetry(recv_time, machine_id, tlm_type, tlm_count, cmd_count, tlm_ms_since_boot, tlm_data)
                break
        except Exception as e:
            return False, e
        return tlm, None

#-----------------Telecommands------------------#

# The basic structure of a telecommand
# The telecommand bytes are capped by Start and End bytes
# cmd_type - The command type of type Enum CmdType
# cmd_data - A list of byte values that mark the command data
#       ! It is up to the telecommand generator to pack these bytes correctly !
class Telecommand():
    def __init__(self, machine_id, cmd_type, cmd_data):
        self.start_byte0 = CMD_START_BYTE_VAL
        self.start_byte1 = CMD_START_BYTE_VAL
        self.destination_machine_id = machine_id
        self.cmd_type = int(cmd_type)
        self.cmd_data = cmd_data
        self.end_byte0 = CMD_END_BYTE_VAL
        self.end_byte1 = CMD_END_BYTE_VAL

    def _to_bytes(self):
        cmd_bytes = [self.start_byte0, self.start_byte1, self.destination_machine_id, self.cmd_type, len(cmd_data)]
        cmd_bytes.extend(self.cmd_data)
        cmd_bytes.extend([self.end_byte0, self.end_byte1])
        return bytes(cmd_bytes)

# Generates command
# Accepts two tuples
#   leftMotor - (pwm_val, motor_action)
#   rightMotor - (pwm_val, motor_action)
#       pwm_val - A byte representing the PWM value that controls motor power
#       motor_action - An Enum MotorAction representing the motor direction
def gen_cmd_motor_ctrl(destination_machine_id, leftMotor, rightMotor):
    if not (0 <= leftMotor[0] <= 255 and 0 <= rightMotor[0] <= 255):
        raise ValueError("pwm_val must be uint8")
    if not (isinstance(leftMotor[1], MotorAction) and isinstance(rightMotor[1], MotorAction)):
        raise ValueError("motor_action must be of type MotorAction")

    return Telecommand(destination_machine_id, CmdType.CMD_MOTOR_CTRL, [leftMotor[0], int(leftMotor[1]), rightMotor[0], int(rightMotor[1])])

#----------------Telemetry-------------------#
class Telemetry():
    def __init__(self, recv_time, machine_id, tlm_type, tlm_count, cmd_count, ms_since_boot, tlm_data):
        self.recv_time = recv_time
        self.machine_id = machine_id
        self.tlm_type = tlm_type
        self.tlm_count = tlm_count
        self.cmd_count = cmd_count
        self.ms_since_boot = ms_since_boot
        self.tlm_data = tlm_data

    def __str__(self):
        if self.tlm_type == TlmType.TLM_ALIVE:
            return "Tlm:: RecvTime: {0}, MachineID: {1}, TlmType: {2}, TlmCnt: {3}, CmdCnt: {4}, MsSinceBoot: {5}" \
                .format(self.recv_time.strftime("%H:%M:%S"), self.machine_id, self.tlm_type, self.tlm_count, self.cmd_count, self.ms_since_boot)
        else:
            return "Not Defined Yet"

def test_tctm():
    print("Testing TCTM...")

    # Test Motor Control Command
    destination_machine_id = MACHINE_ID
    cmd = gen_cmd_motor_ctrl(destination_machine_id, (255, MotorAction.FORWARD), (255, MotorAction.FORWARD))
    assert([97, 97, 68, 3, 4, 255, 2, 255, 2, 98, 98] == list(cmd._to_bytes()))

    # Test TCTM Manager Construction
    tctm_manager = TctmManager()
    try:
        tctm_manager.send_cmd(gen_cmd_motor_ctrl(destination_machine_id, (255, MotorAction.FORWARD), (255, MotorAction.FORWARD)))
    except ValueError:
        pass

    print("Passed TCTM tests.")

test_tctm()