//Doggo
//26 Apr 2020
//Ryan Proffitt

#define DEBUG_PRINT 1

#define HK_TLM_ON 1
#define HK_TLM_LEN 6
#define HK_TLM_START 0x50
#define HK_TLM_END 0x51

#define MOTOR0_FWD_PIN 5
#define MOTOR0_BCK_PIN 4
#define MOTOR1_FWD_PIN 7
#define MOTOR1_BCK_PIN 6

#define MOTOR0_ENABLE_PIN 8
#define MOTOR1_ENABLE_PIN 9

void DebugPrint(char *str){
  if(DEBUG_PRINT == 1){
    Serial.print(str);
  }
}

/*
 * MOTORS
 */
enum MotorAction{
  backward,
  neutral,
  forward
};

typedef struct{
  int motor_id,
  int enable_pin,
  int fwd_pin,
  int bck_pin,
  unsigned int spd
}Motor;

/*
 * COMMS
 */
enum CommStatus{
  OK,
  ERR
};

typedef struct{
  CommStatus raspberrypi_com_sts,
  CommStatus motor0_com_sts,
  CommStatus motor1_com_sts
}Comms;

typedef struct{
  unsigned long last_hk_time,
  Comms comms
}State;

int InitializeComms(Comms *comms){
  int sts = 0;
  sts = Serial.begin(9600);
  if(sts != 0){
    comms.raspberrypi_com_sts = CommStatus ERR;
    DebugPrint("Failure to initialize serial connection. Exiting.\n");
    return 1;
  }

  comms.raspberrypi_com_sts = CommStatus OK;
  DebugPrint("Arduino connected.\n");
  return 0;
}

/*
 * Telemetry
 */
void SendHkTlm(State *state){
  if(HK_TLM_ON && millis() - state->last_hk_time > 1000){
    unsigned byte tlm[HK_TLM_LEN];
    tlm[0] = HK_TLM_START;
    tlm[1] = HK_TLM_START;
    tlm[2] = state->motor0.spd;
    tlm[3] = state->motor1.spd;
    tlm[4] = HK_TLM_END;
    tlm[5] = HK_TLM_END;
    
    Serial.write(tlm);
  }
}

void InitializeState(State *state){
  state->last_hk_time = millis();

  //Add init() for motors later. want a check for the hardware
  state->motor0 = {0, MOTOR0_ENABLE_PIN, MOTOR0_FWD_PIN, MOTOR0_BCK_PIN, 128};
  state->motor1 = {1, MOTOR1_ENABLE_PIN, MOTOR1_FWD_PIN, MOTOR1_BCK_PIN, 128};
  
  state->comms = Comms{CommStatus ERR,CommStatus OK, CommStatus OK};
  InitializeComms(&(state->comms));

  //want a loop here in the future to reattempt connection if ever lost
}

State state;
void setup() {
  InitializeState(&state);
}

void loop() {
  //want a system check here to see if any errors occurred in last loop
  SendHkTlm(&state);
}
