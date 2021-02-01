// All-Purpose Robot
// 31 Jan 2021
// Ryan Proffitt
// This is the component module's (Arduino) core logic.

#define HK_TLM_ON 1
#define HK_TLM_START 0x50
#define HK_TLM_END 0x51
#define HK_HEADER_LEN 0x10

#define HK_STX0_IDX 0x00
#define HK_STX1_IDX 0x01
#define HK_MACHINE_ID_IDX 0x02
#define HK_TLM_TYPE_IDX 0x03
#define HK_TLM_COUNT_IDX 0x04
#define HK_SENT_TIME0_IDX 0x06
#define HK_SENT_TIME1_IDX 0x07
#define HK_SENT_TIME2_IDX 0x08
#define HK_SENT_TIME3_IDX 0x09
#define HK_DATA_LEN 0x10
#define HK_TLM_DATA_IDX 0x11

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
  CommStatus pi_com_sts;
  CommStatus motor0_com_sts;
  CommStatus motor1_com_sts;
}Comms;

typedef struct{
  unsigned long last_hk_time;
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

void SendHkTlm(State *state){
  if(HK_TLM_ON && millis() - state->last_hk_time >= 1000){
    state->comms.tlm_cnt += 1;
    byte tlm[HK_TLM_LEN];
    tlm[HK_STX0_IDX] = HK_TLM_START;
    tlm[HK_STX1_IDX] = HK_TLM_START;
    tlm[HK_MACHINE_ID] = state->machine_id;
    tlm[HK_TLM_COUNT] = state->comms.tlm_cnt;
    tlm[HK_MOTOR0_IDX] = state->motor0.pwm_val;
    tlm[HK_MOTOR1_IDX] = state->motor1.pwm_val;
    tlm[HK_SENT_TIME0_IDX] = (state->last_hk_time >> 24) & 0xFF;
    tlm[HK_SENT_TIME1_IDX] = (state->last_hk_time >> 16) & 0xFF;
    tlm[HK_SENT_TIME2_IDX] = (state->last_hk_time >> 8) & 0xFF;
    tlm[HK_SENT_TIME3_IDX] = state->last_hk_time & 0xFF;
    tlm[HK_END0_IDX] = HK_TLM_END;
    tlm[HK_END1_IDX] = HK_TLM_END;
    
    Serial.write(tlm, HK_TLM_LEN);
    state->last_hk_time = millis();
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
  state->last_hk_time = millis();

  state->machine_id = 0x44;

  //Add init() for motors later. want a check for the hardware
  state->motor0 = {0, MOTOR0_ENABLE_PIN, MOTOR0_FWD_PIN, MOTOR0_BCK_PIN, 127};
  state->motor1 = {1, MOTOR1_ENABLE_PIN, MOTOR1_FWD_PIN, MOTOR1_BCK_PIN, 127};
  
  state->comms = {ERR, OK, OK};
  InitializeComms(&(state->comms));
  InitializeMotors(state, true);
}

void loop() {
  SendHkTlm(&state);

  state.motor0.pwm_val = 1;
  state.motor1.pwm_val = 1;
  digitalWrite(MOTOR0_FWD_PIN, LOW);
  digitalWrite(MOTOR0_BCK_PIN, HIGH);
  digitalWrite(MOTOR1_FWD_PIN, LOW);
  digitalWrite(MOTOR1_BCK_PIN, HIGH);
  
  //RunTests(&state, &test_state);
}
