#ifndef __COMMON_H__
#define __COMMON_H__

#define PIPE_MASTER_TO_SLAVE "Got01"
#define PIPE_SLAVE_TO_MASTER "Dum02"

#define VERSION_A   1
#define VERSION_B   0
#define VERSION_C   0

enum Command {
  GET_STATUS = 1,
  OPEN_DOOR,
  CLOSE_DOOR
};

enum StateMachine {
  OPENING = 1,
  OPENED,
  CLOSING,
  CLOSED
};

uint16_t ComputeSecret(uint16_t c) {
  c += 0xA5;
  for (int i = 0; i < 3; ++i)
    c = ((c & 0x8000) >> 15) | (c << 1);

  return c ^ 0xBEAF;
}

struct ChickenHouseStatus {
  uint16_t secret;
  enum StateMachine stateMachine;
  float temperature;
  byte version_a;
  byte version_b;
  byte version_c;
};

struct ChickenHouseCommand {
  uint16_t secret;
  enum Command command;
};



#endif