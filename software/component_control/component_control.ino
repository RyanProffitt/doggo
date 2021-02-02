// All-Purpose Robot
// 31 Jan 2021
// Ryan Proffitt
// This is the component module's (Arduino) core logic.

// General Macros
#define CMD_START_BYTE_VAL 0x60
#define CMD_END_BYTE_VAL 0x61

#define CMD_NOP 0x00
#define CMD_STS 0x01
#define CMD_SYSTEM_CHECK 0x02
#define CMD_MOTOR_CTRL 0x03
#define CMD_RESERVED 0xFF

#define CMD_EXEC_STS_OK 0
#define CMD_EXEC_STS_DOMAIN_ERR 1
#define CMD_EXEC_STS_DATA_LEN_ERR 2
#define CMD_EXEC_STS_INVALID_CMD 3

#define TLM_START_BYTE_VAL 0x50
#define TLM_END_BYTE_VAL 0x51

#define TLM_ALIVE 0x00
#define TLM_STS_RES 0x01
#define TLM_SYSTEM_CHECK_RES 0x02
#define TLM_MOTOR_CTRL_RES 0x03

// Command Macros
#define MAX_CMD_DATA_LEN 256

// Telemetry Macros
#define TLM_ON 1
#define TLM_HEADER_LEN 11

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

//left motor
#define MOTOR0_ENABLE_PIN 11
#define MOTOR0_FWD_PIN 5
#define MOTOR0_BCK_PIN 6

//right motor
#define MOTOR1_ENABLE_PIN 10
#define MOTOR1_FWD_PIN 3
#define MOTOR1_BCK_PIN 4

enum MotorTestState{
  DEMO_BACKWARD,
  DEMO_FORWARD
};

enum MotorAction{
  BACKWARD,
  NEUTRAL,
  FORWARD
};

typedef struct{
  int motor_id;
  int enable_pin;
  int fwd_pin;
  int bck_pin;
  byte pwm_val;
}Motor;

enum CommStatus{
  OK,
  ERR
};

typedef struct{
  byte cmd_type;
  byte data_len;
  byte data[MAX_CMD_DATA_LEN];
}RecvdCmd;

typedef struct{
  byte tlm_cnt;
  byte cmd_cnt;
  CommStatus pi_com_sts;
  CommStatus motor0_com_sts;
  CommStatus motor1_com_sts;
}Comms;

typedef struct{
  unsigned long last_TLM_time;
  byte machine_id;
  Motor motor0;
  Motor motor1;
  Comms comms;
}State;

typedef struct{
  //Movement Test
  MotorTestState motor_test_state;
  MotorAction motor_action;
  unsigned long last_direction_change_time;
}TestState;

TestState test_state;
State state;
void setup() {
  InitializeTestState(&test_state);
  InitializeState(&state);
}

int InitializeComms(Comms *comms){
  Serial.begin(9600);
  //in future i want this to ping pi and look for return
  //then keep trying if no return ping

  comms->pi_com_sts = OK;
  comms->tlm_cnt = 0;
  comms->cmd_cnt = 0;
  Serial.write("Arduino connected.\n");
  return 0;
}

void ChangeMotorAction(Motor *motor, byte pwm_val, MotorAction motor_action){
  switch(motor_action){
   case BACKWARD:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, HIGH);
      break;
    case NEUTRAL:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, LOW);
      break;
    case FORWARD:
      motor->pwm_val = pwm_val;
      digitalWrite(motor->fwd_pin, HIGH);
      digitalWrite(motor->bck_pin, LOW);
      break; 
  }
}

void SendTlmAlive(State *state){
  if(TLM_ON && millis() - state->last_TLM_time >= 1000){
    state->comms.tlm_cnt += 1;
    const byte tlm_len = 12;
    byte tlm[tlm_len];
    tlm[TLM_STX0_IDX] = TLM_START_BYTE_VAL;
    tlm[TLM_STX1_IDX] = TLM_START_BYTE_VAL;
    tlm[TLM_MACHINE_ID_IDX] = state->machine_id;
    tlm[TLM_TYPE_IDX] = TLM_ALIVE;
    tlm[TLM_COUNT_IDX] = state->comms.tlm_cnt;
    tlm[TLM_CMD_COUNT_IDX] = state->comms.cmd_cnt;
    tlm[TLM_SENT_TIME0_IDX] = (state->last_TLM_time >> 24) & 0xFF;
    tlm[TLM_SENT_TIME1_IDX] = (state->last_TLM_time >> 16) & 0xFF;
    tlm[TLM_SENT_TIME2_IDX] = (state->last_TLM_time >> 8) & 0xFF;
    tlm[TLM_SENT_TIME3_IDX] = state->last_TLM_time & 0xFF;
    tlm[TLM_DATA_LEN_IDX] = 0;
    tlm[11] = TLM_END_BYTE_VAL;
    tlm[12] = TLM_END_BYTE_VAL;
    
    Serial.write(tlm, tlm_len);
    state->last_TLM_time = millis();
  }
}

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

#define CMD_NOP 0x00
#define CMD_STS 0x01
#define CMD_SYSTEM_CHECK 0x02
#define CMD_MOTOR_CTRL 0x03
#define CMD_RESERVED 0xFF
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

    InterpretCmd(state, &cmd);
    break;
  }
}

// Test Overview
// This code will command the bot to move forward for 500ms, stop, move backwards,
//    stop, move forwards... and on and on...
void motor_test(State *state, TestState *test_state){
  unsigned long tmp_time = millis();
  if(tmp_time - test_state->last_direction_change_time >= 500){
    test_state->last_direction_change_time = tmp_time;
    
    if(test_state->motor_test_state == DEMO_FORWARD){
      switch(test_state->motor_action){
        case NEUTRAL:
          ChangeMotorAction(&(state->motor0), 175, FORWARD);
          ChangeMotorAction(&(state->motor1), 175, FORWARD);
        case FORWARD:
          test_state->motor_test_state = DEMO_BACKWARD;
          ChangeMotorAction(&(state->motor0), 175, NEUTRAL);
          ChangeMotorAction(&(state->motor1), 175, NEUTRAL);
      }
    }else if(test_state->motor_test_state == DEMO_BACKWARD){
      switch(test_state->motor_action){
        case NEUTRAL:
          ChangeMotorAction(&(state->motor0), 175, BACKWARD);
          ChangeMotorAction(&(state->motor1), 175, BACKWARD);
        case FORWARD:
          ChangeMotorAction(&(state->motor0), 175, NEUTRAL);
          ChangeMotorAction(&(state->motor1), 175, NEUTRAL); 
      }
    }
  }
}

void RunTests(State *state, TestState *test_state){
  motor_test(state, test_state);
}

int InitializeMotors(State *state, bool enable_pwm_control){
  state->motor0.motor_id = 0;
  state->motor0.fwd_pin = MOTOR0_FWD_PIN;
  state->motor0.bck_pin = MOTOR0_BCK_PIN;
  state->motor0.pwm_val = 0;

  state->motor1.motor_id = 1;
  state->motor1.fwd_pin = MOTOR1_FWD_PIN;
  state->motor1.bck_pin = MOTOR1_BCK_PIN;
  state->motor1.pwm_val = 0;

  //allows for controlling speed
  if(enable_pwm_control){
    state->motor0.enable_pin = MOTOR0_ENABLE_PIN;
    state->motor1.enable_pin = MOTOR1_ENABLE_PIN;
  }
}

void InitializeTestState(TestState *test_state){
  test_state->last_direction_change_time = millis();
  test_state->motor_test_state = DEMO_FORWARD;
}

void InitializeState(State *state){
  state->last_TLM_time = millis();

  state->machine_id = 0x44;

  //Add init() for motors later. want a check for the hardware
  state->motor0 = {0, MOTOR0_ENABLE_PIN, MOTOR0_FWD_PIN, MOTOR0_BCK_PIN, 0};
  state->motor1 = {1, MOTOR1_ENABLE_PIN, MOTOR1_FWD_PIN, MOTOR1_BCK_PIN, 0};
  
  state->comms = {ERR, OK, OK};
  InitializeComms(&(state->comms));
  InitializeMotors(state, true);
}

void loop() {
  SendTlmAlive(&state);
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
