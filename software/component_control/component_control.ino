// All-Purpose Robot
// 31 Jan 2021
// Ryan Proffitt
// This is the component module's (Arduino) core logic.

// General Macros
#define CMD_NOP 0x00
#define CMD_STS 0x01
#define CMD_SYSTEM_CHECK 0x02
#define CMD_MOTOR_CTRL 0x03

#define TLM_ALIVE 0x00
#define TLM_STS_RES 0x01
#define TLM_SYSTEM_CHECK_RES 0x02
#define TLM_MOTOR_CTRL_RES 0x03

// Telemetry Macros
#define TLM_ON 1
#define TLM_START 0x50
#define TLM_END 0x51
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

void ChangeMotorAction(Motor *motor, MotorAction motor_action){
  switch(motor_action){
   case BACKWARD:
      motor->pwm_val = 0;
      //TODO: update PWM pin HERE
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, HIGH);
      break;
    case NEUTRAL:
      motor->pwm_val = 127;
      //TODO: update PWM pin HERE
      digitalWrite(motor->fwd_pin, LOW);
      digitalWrite(motor->bck_pin, LOW);
      break;
    case FORWARD:
      motor->pwm_val = 255;
      //TODO: update PWM pin HERE
      digitalWrite(motor->fwd_pin, HIGH);
      digitalWrite(motor->bck_pin, LOW);
      break; 
  }
}

# 
void SendTlmAlive(State *state){
  if(TLM_ON && millis() - state->last_TLM_time >= 1000){
    state->comms.tlm_cnt += 1;
    const byte tlm_len = 12;
    byte tlm[tlm_len];
    tlm[TLM_STX0_IDX] = TLM_START;
    tlm[TLM_STX1_IDX] = TLM_START;
    tlm[TLM_MACHINE_ID_IDX] = state->machine_id;
    tlm[TLM_TYPE_IDX] = TLM_ALIVE;
    tlm[TLM_COUNT_IDX] = state->comms.tlm_cnt;
    tlm[TLM_CMD_COUNT_IDX] = state->comms.cmd_cnt;
    tlm[TLM_SENT_TIME0_IDX] = (state->last_TLM_time >> 24) & 0xFF;
    tlm[TLM_SENT_TIME1_IDX] = (state->last_TLM_time >> 16) & 0xFF;
    tlm[TLM_SENT_TIME2_IDX] = (state->last_TLM_time >> 8) & 0xFF;
    tlm[TLM_SENT_TIME3_IDX] = state->last_TLM_time & 0xFF;
    tlm[TLM_DATA_LEN_IDX] = 0;
    tlm[11] = TLM_END;
    tlm[12] = TLM_END;
    
    Serial.write(tlm, tlm_len);
    state->last_TLM_time = millis();
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
          ChangeMotorAction(&(state->motor0), FORWARD);
          ChangeMotorAction(&(state->motor1), FORWARD);
        case FORWARD:
          test_state->motor_test_state = DEMO_BACKWARD;
          ChangeMotorAction(&(state->motor0), NEUTRAL);
          ChangeMotorAction(&(state->motor1), NEUTRAL);
      }
    }else if(test_state->motor_test_state == DEMO_BACKWARD){
      switch(test_state->motor_action){
        case NEUTRAL:
          ChangeMotorAction(&(state->motor0), BACKWARD);
          ChangeMotorAction(&(state->motor1), BACKWARD);
        case FORWARD:
          ChangeMotorAction(&(state->motor0), NEUTRAL);
          ChangeMotorAction(&(state->motor1), NEUTRAL); 
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

  delay(500);
  state.motor0.pwm_val = 75;
  state.motor1.pwm_val = 75;
  analogWrite(MOTOR0_ENABLE_PIN, 75);
  analogWrite(MOTOR1_ENABLE_PIN, 75);
  digitalWrite(MOTOR0_FWD_PIN, LOW);
  digitalWrite(MOTOR0_BCK_PIN, HIGH);
  digitalWrite(MOTOR1_FWD_PIN, LOW);
  digitalWrite(MOTOR1_BCK_PIN, HIGH);

  delay(500);
  analogWrite(MOTOR0_ENABLE_PIN, 75);
  analogWrite(MOTOR1_ENABLE_PIN, 75);
  digitalWrite(MOTOR0_FWD_PIN, HIGH);
  digitalWrite(MOTOR0_BCK_PIN, LOW);
  digitalWrite(MOTOR1_FWD_PIN, HIGH);
  digitalWrite(MOTOR1_BCK_PIN, LOW);

  delay(500);
  analogWrite(MOTOR0_ENABLE_PIN, 75);
  analogWrite(MOTOR1_ENABLE_PIN, 75);
  digitalWrite(MOTOR0_FWD_PIN, HIGH);
  digitalWrite(MOTOR0_BCK_PIN, LOW);
  digitalWrite(MOTOR1_FWD_PIN, HIGH);
  digitalWrite(MOTOR1_BCK_PIN, LOW);
  
  //RunTests(&state, &test_state);
}
