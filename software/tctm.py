# All-Purpose Robot
# June 4 2021
# Ryan Proffitt
# This module contains all of the functionality for generating Doggo telecommands.

import serial
from enum import IntEnum
import time
from queue import Queue

NOISY_DEBUG = True

TEST_MACHINE_ID = 0x44

#----------------TCTM Definitions-------------------#
TLM_HEADER_SIZE = 10
TLM_TRAILER_SIZE = 2

TLM_START_BYTE_VAL = 0x50
TLM_END_BYTE_VAL = 0x51
CMD_START_BYTE_VAL = 0x60
CMD_END_BYTE_VAL = 0x61

class TlmType(IntEnum):
    TLM_HK = 0
    TLM_ACK = 1
TELEMETRY_TYPES = [t.value for t in set(TlmType)]

class CmdType(IntEnum):
    CMD_NOP = 0
    CMD_MACHINE_STS = 1
    CMD_NAVIGATION_CHECK = 2
    CMD_MOTOR_CTRL = 3
    CMD_SYNC_TIME = 4
    CMD_RESERVED = 5
CmdType_VALUES = [c.value for c in set(CmdType)]

# Manages Tctm
# Maintains the serial connection, sends commands, and handles telemetry
class SerialTctmManager:
    def __init__(self, machine_id, serial_port_str="/dev/ttyACM0", baud_rate=9600, custom_timeout=None):
        self.machine_id = machine_id
        self.my_serial_conn = serial.Serial(serial_port_str, baud_rate, timeout=custom_timeout)
        self.registered_machines = {}
        self.tlm_queue = Queue()

        self.last_time_recvd_hk_tlm = 0

        self.last_time_sent_cmd = 0

        self.tlm_cnt = 0
        self.cmd_cnt = 0

    # def send_cmd(self, cmd):
    #     isinstance(cmd, Telecommand)
    #     try:
    #         self.my_serial_conn.write(cmd._to_bytes())
    #     except Exception as e:
    #         if self.my_serial_conn == None:
    #             raise ValueError("Serial connection was never initialized.")
    #         else:
    #             raise e

    # This non-blocking function checks the serial connection for waiting bytes,
    #   gets those bytes, converts each to int, and returns a list of the ints.
    # Returns a list of ints, or -1 if requested amount of bytes not available in serial.
    def get_bytes_as_int_list(self, num_to_get):
        if self.my_serial_conn.inWaiting() > num_to_get:
            incoming_list = []
            for i in range(num_to_get):
                incoming_list.append(int.from_bytes(self.my_serial_conn.read(), "big"))
            return incoming_list
        else:
            return None
    
    # This non-blocking function checks the serial connection for a waiting byte,
    #   gets that single byte, converts it to int, and returns it.
    # Returns the byte, or -1 if no byte available in serial.
    def get_single_byte_as_int(self):
        if self.my_serial_conn.inWaiting() > 0:
            return int.from_bytes(self.my_serial_conn.read(), "big")
        else:
            return -1

    def _extract_tlm(self):
        header_bytes = self.my_serial_conn.read(TLM_HEADER_SIZE)

        assert header_bytes[0] == TLM_START_BYTE_VAL

        # print([int(byt) for byt in header_bytes])

        recv_time = time.time()

        tlm_data_len = int(header_bytes[TLM_HEADER_SIZE - 1])
        data_bytes = self.my_serial_conn.read(tlm_data_len)

        trailer_bytes = self.my_serial_conn.read(TLM_TRAILER_SIZE)

        if trailer_bytes[1] != TLM_END_BYTE_VAL:
            raise ValueError("Unexpected end byte value =", trailer_bytes[1])
        
        # Verify the checksum value
        tlm_check_sum = trailer_bytes[0]
        expected_checksum = (sum(header_bytes[1:]) + sum(data_bytes)) % 256
        if expected_checksum != tlm_check_sum:
            raise ValueError("Expected checksum={}, got {}", expected_checksum, tlm_check_sum)

        tlm_type = header_bytes[2]
        if tlm_type == TlmType.TLM_HK.value:
            self.tlm_queue.put(HkTelemetry(recv_time, header_bytes, data_bytes))
        else:
            print("Telemetry code", tlm_type, "not implemented.")

    # Looks for start bytes on the line
    #   Will fill the Telemetry Queue as telemetry frames are found
    #   Otherwise, does nothing
    def recv(self):
        try:
            self._extract_tlm()
        except Exception as e:
            if NOISY_DEBUG: print(str(e))
        
        if not self.tlm_queue.empty():
            print(self.tlm_queue.get())

#----------------Telemetry-------------------#
# This class contains Telemetry information.
# Please see the TCTM Definition document for structure.
# This excludes the start and end bytes.
class Telemetry:
    def __init__(self, recv_time, header_bytes, data_bytes):
        self.recv_time = recv_time
        self.header_bytes = header_bytes
        self.data_bytes = data_bytes

class HkTelemetry(Telemetry):
    def __init__(self, recv_time, header_bytes, data_bytes):
        Telemetry.__init__(self, recv_time, header_bytes, data_bytes)
        data = None
    
    def __str__(self):
        return "{}: {}".format(str(self.recv_time), str([int(byt) for byt in (self.header_bytes + self.data_bytes)]))

# class CmdAckTelemetry(Telemetry):
#     def __init__(self, recv_time, header_bytes, data_bytes):
#         super.__init__(self, recv_time, header_bytes, data_bytes)
#         data = None

#         print("CmdAckTelemetry not implemented yet.")

#-----------------Telecommands------------------#

# The basic structure of a telecommand
# The telecommand bytes are capped by Start and End bytes
# cmd_type - The command type of type Enum CmdType
# cmd_data - A list of byte values that mark the command data
#       ! It is up to the telecommand generator to pack these bytes correctly !
# class Telecommand:
#     def __init__(self, machine_id, cmd_type, cmd_data):
#         self.start_byte0 = CMD_START_BYTE_VAL
#         self.start_byte1 = CMD_START_BYTE_VAL
#         self.destination_machine_id = machine_id
#         self.cmd_type = int(cmd_type)
#         self.cmd_data_len = len(cmd_data)
#         self.cmd_data = cmd_data
#         self.end_byte0 = CMD_END_BYTE_VAL
#         self.end_byte1 = CMD_END_BYTE_VAL

#     def _to_bytes(self):
#         cmd_bytes = [self.start_byte0, self.start_byte1, self.destination_machine_id, self.cmd_type, self.cmd_data_len]
#         cmd_bytes.extend(self.cmd_data)
#         cmd_bytes.extend([self.end_byte0, self.end_byte1])
#         return bytes(cmd_bytes)

# Generates command
# Accepts two tuples
#   leftMotor - (pwm_val, motor_action)
#   rightMotor - (pwm_val, motor_action)
#       pwm_val - A byte representing the PWM value that controls motor power
#       motor_action - An Enum MotorAction representing the motor direction
# def gen_cmd_motor_ctrl(destination_machine_id, left_motor_pwm, left_motor_action, right_motor_pwm, right_motor_action):
#     if not (0 <= left_motor_pwm <= 255 and 0 <= right_motor_pwm <= 255):
#         raise ValueError("pwm_val must be uint8")
#     if not (isinstance(left_motor_action, MotorAction) and isinstance(right_motor_action, MotorAction)):
#         raise ValueError("motor_action must be of type MotorAction")

#     return Telecommand(destination_machine_id, CmdType.CMD_MOTOR_CTRL, [left_motor_pwm, int(left_motor_action), right_motor_pwm, int(left_motor_action)])

def test_rcv():
    tctm_manager = SerialTctmManager(TEST_MACHINE_ID)
    
    while True:
        tctm_manager.recv()
        time.sleep(.1)

        if not tctm_manager.tlm_queue.empty():
            print(tctm_manager.tlm_queue.get())

# def test_tctm():
#     print("Testing TCTM...")

#     # HK Tlm Test
#     # tlm_bytes = bytearray(bytes([0x50, 0x50, 0x44, 0, 5, 5, 0x00, 0x00, 0x00, 0xFF, 0, 0x51, 0x51]))
#     # test = int.from_bytes(tlm_bytes[2], "big")
#     # #tlm = Telemetry(datetime.now(), (int)tlm_bytes[2], (TlmType)tlm_bytes[2], (int)tlm_bytes[4], (int)tlm_bytes[5], (int)tlm_bytes[2], tlm_data_len, tlm_data)

#     # Test Motor Control Command
#     destination_machine_id = MACHINE_ID
#     cmd = gen_cmd_motor_ctrl(destination_machine_id, 255, MotorAction.FORWARD, 255, MotorAction.FORWARD)
#     assert([96, 96, 68, 3, 4, 255, 2, 255, 2, 97, 97] == list(cmd._to_bytes()))

#     # Test TCTM Manager Construction
#     tctm_manager = TctmManager()
#     try:
#         tctm_manager.send_cmd(gen_cmd_motor_ctrl(destination_machine_id, 255, MotorAction.FORWARD, 255, MotorAction.FORWARD))
#     except ValueError:
#         pass

#     print("Passed TCTM tests.")

#test_tctm()
test_rcv()