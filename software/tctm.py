# All-Purpose Robot
# February 1 2021
# Ryan Proffitt
# This module contains all of the functionality for generating Doggo telecommands.

#TODO: Put some of this in C

import serial
from enum import IntEnum
import time

#----------------Hardware Definitions : Motors-------------------#
class MotorAction(IntEnum):
    BACKWARD = 0
    NEUTRAL = 1
    FORWARD = 2

#----------------Hardware Definitions : Communications-------------------#
MACHINE_ID = 0x44

TIMEOUT_FIFTY_MS = 0.050

#----------------Telemetry Definitions-------------------#
TLM_START_BYTE_VAL = 0x50
TLM_END_BYTE_VAL = 0x51

TLM_NUM_START_BYTES = 2
TLM_HEADER_SIZE = 11
TLM_NUM_ENDER_BYTES = 2

class TlmType(IntEnum):
    TLM_TYPE_HK = 0
    TLM_TYPE_CMD_ACK = 1
TlmType_VALUES = [t.value for t in set(TlmType)]


#----------------Command Definitions-------------------#
CMD_START_BYTE_VAL = 0x60
CMD_END_BYTE_VAL = 0x61

class CmdType(IntEnum):
    CMD_NOP = 0
    CMD_MACHINE_STS = 1
    CMD_NAVIGATION_CHECK = 2
    CMD_MOTOR_CTRL = 3
    CMD_RESERVED = 4
CmdType_VALUES = [c.value for c in set(CmdType)]

# Manages Tctm
# Maintains the serial connection, sends commands, and handles telemetry
class TctmManager:
    #TODO Make all command generators a function of this class
    def __init__(self, serial_port_str="/dev/ttyACM0", baud_rate=9600, custom_timeout=0.1):
        self.my_serial_conn = serial.Serial(serial_port_str, baud_rate, timeout=custom_timeout)
        self.tlm_cnt = 0
        self.cmd_cnt = 0
        self.last_time_recvd_any_tlm = 0
        self.last_time_recvd_hk_tlm = 0

        self._have_start_bytes = False
        self._have_header_bytes = False
        self._have_data_and_end_bytes = False

        self.curr_tlm = Telemetry(time.time(),0,0,0,0,0,0,0)

        self._have_start_byte0 = False
        self._have_start_byte1 = False
        self._tlm_packet_wout_caps = []
        self._have_end_byte0 = False
        self._have_end_byte1 = False

    def send_cmd(self, cmd):
        isinstance(cmd, Telecommand)
        try:
            self.my_serial_conn.write(cmd._to_bytes())
        except Exception as e:
            if self.my_serial_conn == None:
                raise ValueError("Serial connection was never initialized.")
            else:
                raise e

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

    def recv_tlm(self):
            try:
                if not self._have_start_bytes:
                    # Check for Start Bytes
                    if self.get_single_byte_as_int() != TLM_START_BYTE_VAL or self.get_single_byte_as_int() != TLM_START_BYTE_VAL:
                        return None, None
                    self._have_start_bytes = True

                    # Got Start Bytes - Record the Time to Check Against for Timeout Later
                    self.curr_tlm.recv_time = time.time()
                if not self._have_header_bytes:
                    # Check for Header Bytes
                    if not self.my_serial_conn.inWaiting() > TLM_HEADER_SIZE - TLM_NUM_START_BYTES:
                        # Check for Timeout
                        if time.time() - self.curr_tlm.recv_time >= TIMEOUT_FIFTY_MS:
                            print("\tTimeout while waiting for header bytes")
                            return None, TimeoutError
                        return None, None
                    self._have_header_bytes = True

                    # Got Header Bytes - Set Header Bytes
                    self.curr_tlm.machine_id = self.get_single_byte_as_int()
                    self.curr_tlm.tlm_type = self.get_single_byte_as_int()
                    self.curr_tlm.tlm_count = self.get_single_byte_as_int()
                    self.curr_tlm.cmd_count = self.get_single_byte_as_int()
                    self.curr_tlm.ms_since_boot = (self.get_single_byte_as_int() << 24) + \
                        (self.get_single_byte_as_int() << 16) + (self.get_single_byte_as_int() << 8) + self.get_single_byte_as_int()
                    self.curr_tlm.tlm_data_len = self.get_single_byte_as_int()
                if not self._have_data_and_end_bytes:
                    # Check that there is enough data for Data and End Bytes
                    if not self.my_serial_conn.inWaiting() >= self.curr_tlm.tlm_data_len + TLM_NUM_ENDER_BYTES:
                        # Check for Timeout
                            if time.time() - self.curr_tlm.recv_time >= TIMEOUT_FIFTY_MS:
                                return None, TimeoutError
                            return None, None

                    # Got enough bytes for Data and End Bytes - Set Data
                    self.curr_tlm.data = self.get_bytes_as_int_list(self.curr_tlm.tlm_data_len)

                    # Check End Bytes
                    if self.get_single_byte_as_int() != TLM_END_BYTE_VAL or self.get_single_byte_as_int() != TLM_END_BYTE_VAL:
                        print("Malformed packet, returning error")
                        return None, ValueError("Malformed Packet")

                    self._have_start_bytes = False
                    self._have_header_bytes = False
                    self._have_data_and_end_bytes = False

                    return self.curr_tlm, None
            except Exception as e:
                return None, e

#----------------Telemetry-------------------#
# This class contains Telemetry information.
# Please see the TCTM Definition document for structure.
# This excludes the start and end bytes.
class Telemetry:
    def __init__(self, recv_time, machine_id, tlm_type, tlm_count, cmd_count, ms_since_boot, tlm_data_len, tlm_data):
        self.recv_time = recv_time
        self.machine_id = machine_id
        self.tlm_type = tlm_type
        self.tlm_count = tlm_count
        self.cmd_count = cmd_count
        self.ms_since_boot = ms_since_boot
        self.tlm_data_len = tlm_data_len
        self.tlm_data = tlm_data

    def __str__(self):
        return "Tlm:: RecvTime: {0}, MachineID: {1}, TlmType: {2}, TlmCnt: {3}, CmdCnt: {4}, MsSinceBoot: {5}, DataLen: {6}, Data: {7}" \
                .format(self.recv_time, self.machine_id, TlmType(self.tlm_type).name, self.tlm_count, self.cmd_count, self.ms_since_boot, self.tlm_data_len, str(self.tlm_data))

#-----------------Telecommands------------------#

# The basic structure of a telecommand
# The telecommand bytes are capped by Start and End bytes
# cmd_type - The command type of type Enum CmdType
# cmd_data - A list of byte values that mark the command data
#       ! It is up to the telecommand generator to pack these bytes correctly !
class Telecommand:
    def __init__(self, machine_id, cmd_type, cmd_data):
        self.start_byte0 = CMD_START_BYTE_VAL
        self.start_byte1 = CMD_START_BYTE_VAL
        self.destination_machine_id = machine_id
        self.cmd_type = int(cmd_type)
        self.cmd_data_len = len(cmd_data)
        self.cmd_data = cmd_data
        self.end_byte0 = CMD_END_BYTE_VAL
        self.end_byte1 = CMD_END_BYTE_VAL

    def _to_bytes(self):
        cmd_bytes = [self.start_byte0, self.start_byte1, self.destination_machine_id, self.cmd_type, self.cmd_data_len]
        cmd_bytes.extend(self.cmd_data)
        cmd_bytes.extend([self.end_byte0, self.end_byte1])
        return bytes(cmd_bytes)

# Generates command
# Accepts two tuples
#   leftMotor - (pwm_val, motor_action)
#   rightMotor - (pwm_val, motor_action)
#       pwm_val - A byte representing the PWM value that controls motor power
#       motor_action - An Enum MotorAction representing the motor direction
def gen_cmd_motor_ctrl(destination_machine_id, left_motor_pwm, left_motor_action, right_motor_pwm, right_motor_action):
    if not (0 <= left_motor_pwm <= 255 and 0 <= right_motor_pwm <= 255):
        raise ValueError("pwm_val must be uint8")
    if not (isinstance(left_motor_action, MotorAction) and isinstance(right_motor_action, MotorAction)):
        raise ValueError("motor_action must be of type MotorAction")

    return Telecommand(destination_machine_id, CmdType.CMD_MOTOR_CTRL, [left_motor_pwm, int(left_motor_action), right_motor_pwm, int(left_motor_action)])

def test_rcv():
    tctm_manager = TctmManager()
    
    while True:
        tlm, err = tctm_manager.recv_tlm()

        if tlm:
            print(tlm)
        elif err:
            print(err)

def test_tctm():
    print("Testing TCTM...")

    # HK Tlm Test
    # tlm_bytes = bytearray(bytes([0x50, 0x50, 0x44, 0, 5, 5, 0x00, 0x00, 0x00, 0xFF, 0, 0x51, 0x51]))
    # test = int.from_bytes(tlm_bytes[2], "big")
    # #tlm = Telemetry(datetime.now(), (int)tlm_bytes[2], (TlmType)tlm_bytes[2], (int)tlm_bytes[4], (int)tlm_bytes[5], (int)tlm_bytes[2], tlm_data_len, tlm_data)

    # Test Motor Control Command
    destination_machine_id = MACHINE_ID
    cmd = gen_cmd_motor_ctrl(destination_machine_id, 255, MotorAction.FORWARD, 255, MotorAction.FORWARD)
    assert([96, 96, 68, 3, 4, 255, 2, 255, 2, 97, 97] == list(cmd._to_bytes()))

    # Test TCTM Manager Construction
    tctm_manager = TctmManager()
    try:
        tctm_manager.send_cmd(gen_cmd_motor_ctrl(destination_machine_id, 255, MotorAction.FORWARD, 255, MotorAction.FORWARD))
    except ValueError:
        pass

    print("Passed TCTM tests.")

#test_tctm()
#test_rcv()