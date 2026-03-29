#ifndef __XBOX_H
#define __XBOX_H

#include "main.h"

#define maxJoy_dev2 0x7fff
#define maxTrig 0x3ff

#define XBOX_CONTROLLER_INDEX_BUTTONS_DIR 12
#define XBOX_CONTROLLER_INDEX_BUTTONS_MAIN 13
#define XBOX_CONTROLLER_INDEX_BUTTONS_CENTER 14
#define XBOX_CONTROLLER_INDEX_BUTTONS_SHARE 15


typedef struct xbox
{
  uint8_t btnA, btnB, btnX, btnY;
  uint8_t btnShare, btnStart, btnSelect, btnXbox;
  uint8_t btnShare_last, btnStart_last, btnSelect_last, btnXbox_last;
  uint8_t btnShare_falling, btnStart_falling, btnSelect_falling, btnXbox_falling;
  // side top button
  uint8_t btnLB, btnRB;
  // button on joy stick
  uint8_t btnLS, btnRS;
  uint8_t btnDirUp, btnDirLeft, btnDirRight, btnDirDown;
  int16_t joyLHori ;
  int16_t joyLVert ;
  int16_t joyRHori ;
  int16_t joyRVert ;
  uint16_t trigLT, trigRT;
}_xbox_t;
extern _xbox_t xbox;
void Xbox_rx_buf_Handle(uint8_t *rx_buffer);

#endif