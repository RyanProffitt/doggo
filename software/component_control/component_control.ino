// All-Purpose Robot
// 31 Jan 2021
// Ryan Proffitt
// This is the component module's (Arduino) core logic.

// Hardware Definitions : Motors //
enum MotorAction{
  MOTOR_ACTION_BACKWARD,
  MOTOR_ACTION_NEUTRAL,
  MOTOR_ACTION_FORWARD,
};

#define MOTOR0_ENABLE_PIN 11 // Left Motor
#define MOTOR0_FWD_PIN 5
#define MOTOR0_BCK_PIN 6

#define MOTOR1_ENABLE_PIN 10 // Right Motor
#define MOTOR1_FWD_PIN 3
#define MOTOR1_BCK_PIN 4

typedef struct{
  int motor_id;
  int enable_pin;
  int fwd_pin;
  int bck_pin;
  MotorAction action;
  byte pwm_val;
}Motor;

// Hardware Definitions : Communications //
#define MACHINE_ID 0x44
#define COMMS_POWER 1

enum CommsHardware{
  SERIAL_CABLE,
  WIFI_UNIT,
  BLUETOOTH_UNIT,
  RADIO_UNIT
};

enum CommsStatus{
  COMMS_OK,
  COMMS_ERR
};

typedef struct{
  byte machine_id;
  byte tlm_cnt;
  byte cmd_cnt;
  CommsHardware comms_hw;
  CommsStatus comms_sts;
  unsigned long last_hk_time_since_boot;
}Comms;

// Telemetry Definitions //
#define TLM_START_BYTE_VAL 0x50
#define TLM_END_BYTE_VAL 0x51

#define TLM_HEADER_SIZE 11
#define TLM_TAIL_SIZE 2

#define TLM_STX0_IDX 0
#define TLM_STX1_IDX 1
#define TLM_MACHINE_ID_IDX 2
#define TLM_TYPE_IDX 3
#define TLM_COUNT_IDX 4
#define TLM_CMD_COUNT_IDX 5
#define TLM_SENT_TIME0_IDX 6
#define TLM_SENT_TIME1_IDX 7
#define TLM_SENT_TIME2_IDX 8
#define TLM_SENT_TIME3_IDX 9
#define TLM_DATA_LEN_IDX 10
#define TLM_DATA_IDX 11

enum TelemetryType{
  TLM_TYPE_HK,
  TLM_TYPE_CMD_ACK,
};
#define TLM_TYPE_HK_DATA_LEN 0

// Command Definitions //
#define CMD_START_BYTE_VAL 0x60
#define CMD_END_BYTE_VAL 0x61

enum CommandType{
  CMD_NOP,
  CMD_MACHINE_STS,
  CMD_NAVIGATION_CHECK,
  CMD_MOTOR_CTRL,
  CMD_RESERVED
};

enum CommandExecutionStatus{
  CMD_EXEC_STS_OK,
  CMD_EXEC_STS_DOMAIN_ERR,
  CMD_EXEC_STS_DATA_LEN_ERR,
  CMD_EXEC_STS_INVALID_CMD
};

typedef struct{
  byte cmd_type;
  byte data_len;
  byte *data;
}RecvdCmd;

// Machine State Definitions //
typedef struct{
  Motor motor0;
  Motor motor1;
  Comms comms;
}State;

void InitializeState(State *state){
  // Motor Init //
  state->motor0 = {0, MOTOR0_ENABLE_PIN, MOTOR0_FWD_PIN, MOTOR0_BCK_PIN, MOTOR_ACTION_NEUTRAL, 0};
  state->motor1 = {1, MOTOR1_ENABLE_PIN, MOTOR1_FWD_PIN, MOTOR1_BCK_PIN, MOTOR_ACTION_NEUTRAL, 0};

  // Communications Init //
  state->comms = {MACHINE_ID, 0, 0, SERIAL_CABLE, COMMS_ERR};
  (state->comms).last_hk_time_since_boot = 0;
  if(COMMS_POWER){
    //TODO: Add the other communications methods as they are added
    switch((state->comms).comms_hw){
      case SERIAL_CABLE:
        Serial.begin(9600);
        break;
    }
    SendTlmHk(state);
  }
}

// Motor Functionality //

// This function changes the direction and power of a given motor.
// Accepts:
//  motor - A pointer to a motor
//  pwm_val - A pulse width modulation value with possible values 0 through 255
// motor_action - The direction of motor rotation
void ChangeMotorAction(Motor *motor, byte pwm_val, MotorAction motor_action){
  switch(motor_action){
   case MOTOR_ACTION_BACKWARD:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, HIGH);
      break;
    case MOTOR_ACTION_NEUTRAL:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, LOW);
      break;
    case MOTOR_ACTION_FORWARD:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, HIGH);
      digitalWrite(motor->bck_pin, LOW);
      break; 
  }
}

// Communications Functionality //

// This function sends HK telemetry via the communication hardware.
// Will automatically update (state->comms).last_hk_time_since_boot value on success.
// Accepts
//  state - The machine state.
void SendTlmHk(State *state){
  CommsStatus sts = SendTlm(state, TLM_TYPE_HK, NULL, TLM_TYPE_HK_DATA_LEN);
  if(sts == COMMS_OK){
    (state->comms).last_hk_time_since_boot = millis();
  }
}

// This function sends telemetry packet via Serial.
// A telemetry packet contains Header, Data Field, and Tail bytes.
// Increments (state->comms).tlm_cnt on success.
// Updates (state->comms).comms_sts every time this is called.
// Accepts
//  state - A pointer to the machine state struct
//  tlm_type - The type of telemetry packet (see the available tlm types under enum TelemetryType)
//  data - The data being sent via telemetry
//  data_len - The length of the data
CommsStatus SendTlm(State *state, TelemetryType tlm_type, byte *data, unsigned int data_len){
  if(COMMS_POWER != 1){
    return COMMS_ERR;
  }

  // Set Header Variables
  byte tlm[TLM_STX0_IDX] = TLM_START_BYTE_VAL;
  byte tlm[TLM_STX1_IDX] = TLM_START_BYTE_VAL;
  byte tlm[TLM_MACHINE_ID_IDX] = (state->comms).machine_id;
  byte tlm[TLM_TYPE_IDX] = (byte)tlm_type;
  byte tlm[TLM_COUNT_IDX] = (state->comms).tlm_cnt;
  tlm[TLM_SENT_TIME0_IDX] = ((state->comms).last_hk_time_since_boot >> 24) & 0xFF;
  tlm[TLM_SENT_TIME1_IDX] = ((state->comms).last_hk_time_since_boot >> 16) & 0xFF;
  tlm[TLM_SENT_TIME2_IDX] = ((state->comms).last_hk_time_since_boot >> 8) & 0xFF;
  tlm[TLM_SENT_TIME3_IDX] = (state->comms).last_hk_time_since_boot & 0xFF;
  tlm[TLM_DATA_LEN_IDX] = data_len;
  
  // Set Data Field
  for(int i = 0; i < data_len; i++){
    tlm[TLM_DATA_IDX + i] = data[i];
  }

  // Set Tail Variables
  tlm[TLM_DATA_IDX + data_len] = TLM_END_BYTE_VAL;
  tlm[TLM_DATA_IDX + data_len + 1] = TLM_END_BYTE_VAL;

  // Send packet and check status
  size_t tlm_packet_size = TLM_HEADER_SIZE + data_len + TLM_TAIL_SIZE);
  if(Serial.write(tlm, tlm_packet_size) == tlm_packet_size){
    (state->comms).tlm_cnt++;
    return COMMS_OK;
  }else{
    return COMMS_ERR;
  }
}

// This function updates the motor states based on the Motor Control command.
// The motor control command data bytes are formatted as {Motor0 PWM, Motor0 Action, Motor1 PWM, Motor1 Action}
// Accepts:
//  state - A pointer to the machine state
//  cmd - A pointer to the command
int CmdExec_MotorCtrl(State *state, RecvdCmd *cmd){
  // Perform Cmd Checks
  if(cmd->data_len != 4){
    return CMD_EXEC_STS_DATA_LEN_ERR;
  }
  if(cmd->data[1] > 2 || cmd->data[3] > 2){
    return CMD_EXEC_STS_DOMAIN_ERR;
  }

  ChangeMotorAction(&(state->motor0), cmd->data[0], (MotorAction)cmd->data[1]);
  ChangeMotorAction(&(state->motor0), cmd->data[2], (MotorAction)cmd->data[3]);

  return CMD_EXEC_STS_OK;
}

// This function 
int InterpretCmd(State *state, RecvdCmd *cmd){
  int cmd_exec_sts = 0;

  //TODO CMD_NOP, CMD_STS, CMD_SYSTEM_CHECK
  switch(cmd->cmd_type){
    case CMD_MOTOR_CTRL:
      cmd_exec_sts = CmdExec_MotorCtrl(state, cmd);
      break;
    default:
      cmd_exec_sts = CMD_EXEC_STS_INVALID_CMD;
      break;
      
    return cmd_exec_sts;
  }
}

// Receive and interpret commands via serial
void RcvCmds(State *state){
  while(Serial.available() != 0){
    // Check for Start Byte #1
    if(Serial.read() != CMD_START_BYTE_VAL){continue;}

    // Check for Start Byte #2
    if(Serial.available() == 0){break;}
    if(Serial.read() != CMD_START_BYTE_VAL){continue;}

    // Check for Machine ID
    if(Serial.available() == 0){break;}
    if(Serial.read() != state->machine_id){continue;}

    RecvdCmd cmd;

    // Get Command Type
    if(Serial.available() == 0){break;}
    cmd.cmd_type = Serial.read();

    // Get Command Data Length
    if(Serial.available() == 0){break;}
    cmd.data_len = Serial.read();

    // Get Command Data
    byte remaining_serial_bytes = Serial.available();
    if(remaining_serial_bytes != 0){break;}
    for(int i = 0; i < remaining_serial_bytes; i++){
      cmd.data[i] = Serial.read();
    }

    // Check End Byte #1
    if(Serial.available() == 0){break;}
    if(Serial.read() != CMD_END_BYTE_VAL){continue;}

    // Check End Byte #2
    if(Serial.available() == 0){break;}
    if(Serial.read() != CMD_END_BYTE_VAL){continue;}

    byte interpret_sts = 0;
    interpret_sts = InterpretCmd(state, &cmd);



    break;
  }
}

void Perform1hzFunctions(State *state){
  if(millis() - state->last_hk_time_since_boot >= 1000){
    SendTlmHk(state);
  }
}

State state;
void setup(){
  InitializeState(&state);
}

void loop(){
  Perform1hzFunctions(&state);
  RcvCmds(&state);

  // delay(500);
  // state.motor0.pwm_val = 75;
  // state.motor1.pwm_val = 75;
  // analogWrite(MOTOR0_ENABLE_PIN, 75);
  // analogWrite(MOTOR1_ENABLE_PIN, 75);
  // digitalWrite(MOTOR0_FWD_PIN, LOW);
  // digitalWrite(MOTOR0_BCK_PIN, HIGH);
  // digitalWrite(MOTOR1_FWD_PIN, LOW);
  // digitalWrite(MOTOR1_BCK_PIN, HIGH);

  // delay(500);
  // analogWrite(MOTOR0_ENABLE_PIN, 75);
  // analogWrite(MOTOR1_ENABLE_PIN, 75);
  // digitalWrite(MOTOR0_FWD_PIN, HIGH);
  // digitalWrite(MOTOR0_BCK_PIN, LOW);
  // digitalWrite(MOTOR1_FWD_PIN, HIGH);
  // digitalWrite(MOTOR1_BCK_PIN, LOW);

  // delay(500);
  // analogWrite(MOTOR0_ENABLE_PIN, 75);
  // analogWrite(MOTOR1_ENABLE_PIN, 75);
  // digitalWrite(MOTOR0_FWD_PIN, HIGH);
  // digitalWrite(MOTOR0_BCK_PIN, LOW);
  // digitalWrite(MOTOR1_FWD_PIN, HIGH);
  // digitalWrite(MOTOR1_BCK_PIN, LOW);
  
  //RunTests(&state, &test_state);
}
