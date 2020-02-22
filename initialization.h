
#include "config.h"

struct configuration conf[2];

void initialization() {
  conf[0].featherCAN = 7;
  conf[0].analogNum = 1;
  conf[0].sN.sN[0] = 0xA97F72CD;
  conf[0].sN.sN[1] = 0x50504335;
  conf[0].sN.sN[2] = 0x382E3120;
  conf[0].sN.sN[3] = 0xFF0E2814;
  conf[0].calib[0].fromLow = 9;
  conf[0].calib[0].fromHigh = 4079;
  conf[0].calib[1].fromLow = 11;
  conf[0].calib[1].fromHigh = 4094;
  conf[0].calib[2].fromLow = 9;
  conf[0].calib[2].fromHigh = 4087;
  conf[0].canId = 0x0a0800C1;
  conf[0].type = TIMEOFFLIGHT;

  conf[1].featherCAN = 8;
  conf[1].analogNum = 2;
  conf[1].sN.sN[0] = 0xB644C4E4;
  conf[1].sN.sN[1] = 0x50504335;
  conf[1].sN.sN[2] = 0x382E3120;
  conf[1].sN.sN[3] = 0xFF0D1A30;
  conf[1].calib[0].fromLow = 10;
  conf[1].calib[0].fromHigh = 4095;
  conf[1].calib[1].fromLow = 11;
  conf[1].calib[1].fromHigh = 4091;
  conf[1].calib[2].fromLow = 11;
  conf[1].calib[2].fromHigh = 4094;
  conf[1].canId = 0x0a080081;
  conf[1].type = SHOOTER;
}
