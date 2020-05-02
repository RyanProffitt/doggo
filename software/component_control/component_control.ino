//Doggo
//26 Apr 2020
//Ryan Proffitt

#define DEBUG_PRINT 1

#define HK_TLM_ON 1
#define HK_TLM_LEN 10
#define HK_TLM_START 0x50
#define HK_TLM_END 0x51

#define HK_STX0_IDX 0
#define HK_STX1_IDX 1
#define HK_MOTOR0_IDX 2
#define HK_MOTOR1_IDX 3
#define HK_SENT_TIME0_IDX 4
#define HK_SENT_TIME1_IDX 5
#define HK_SENT_TIME2_IDX 6
#define HK_SENT_TIME3_IDX 7
#define HK_END0_IDX 8
#define HK_END1_IDX 9

#define MOTOR0_FWD_PIN 5
#define MOTOR0_BCK_PIN 4
#define MOTOR1_FWD_PIN 7
#define MOTOR1_BCK_PIN 6

#define MOTOR0_ENABLE_PIN 8
#define MOTOR1_ENABLE_PIN 9

enum MotorAction{
  backward,
  neutral,
  forward
};

typedef struct{
  int motor_id;
  int enable_pin;
  int fwd_pin;
  int bck_pin;
  byte spd;
}Motor;

enum CommStatus{
  OK,
  ERR
};

typedef struct{
  CommStatus pi_com_sts;
  CommStatus motor0_com_sts;
  CommStatus motor1_com_sts;
}Comms;

typedef struct{
  unsigned long last_hk_time;
  Motor motor0;
  Motor motor1;
  Comms comms;
}State;

int InitializeComms(Comms *comms){
  Serial.begin(9600);
  //in future i want this to ping pi and look for return
  //then keep trying if no return ping

  comms->pi_com_sts = OK;
  Serial.write("Arduino connected.\n");
  return 0;
}

void SendHkTlm(State *state){
  if(HK_TLM_ON && millis() - state->last_hk_time > 1000){
    byte  tlm[HK_TLM_LEN];
    tlm[HK_STX0_IDX] = HK_TLM_START;
    tlm[HK_STX1_IDX] = HK_TLM_START;
    tlm[HK_MOTOR0_IDX] = state->motor0.spd;
    tlm[HK_MOTOR1_IDX] = state->motor1.spd;
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

void InitializeState(State *state){
  state->last_hk_time = millis();

  //Add init() for motors later. want a check for the hardware
  state->motor0 = {0, MOTOR0_ENABLE_PIN, MOTOR0_FWD_PIN, MOTOR0_BCK_PIN, 128};
  state->motor1 = {1, MOTOR1_ENABLE_PIN, MOTOR1_FWD_PIN, MOTOR1_BCK_PIN, 128};
  
  state->comms = {ERR, OK, OK};
  InitializeComms(&(state->comms));
}

State state;
void setup() {
  InitializeState(&state);
}

void loop() {
  SendHkTlm(&state);
}
