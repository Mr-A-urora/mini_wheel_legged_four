#include "xbox.h"

_xbox_t xbox;
volatile uint8_t rx_buf[16];

void Xbox_rx_buf_Handle(uint8_t *rx_buffer)
{
	uint8_t i;

	for (i = 0; i < 16; i++){
		rx_buf[i] = rx_buffer[i];
	}
	uint8_t btnBits;
	btnBits = rx_buf[XBOX_CONTROLLER_INDEX_BUTTONS_MAIN];
	xbox.btnA = (btnBits & 0b00000001) >> 0;
	xbox.btnB = (btnBits & 0b00000010) >> 1;
	xbox.btnX = (btnBits & 0b00001000) >> 3;
	xbox.btnY = (btnBits & 0b00010000) >> 4;
	xbox.btnLB = (btnBits & 0b01000000) >> 6;
	xbox.btnRB = (btnBits & 0b10000000) >> 7;
	btnBits = rx_buf[XBOX_CONTROLLER_INDEX_BUTTONS_CENTER];
	xbox.btnSelect = (btnBits & 0b00000100) >> 2;
	xbox.btnStart = (btnBits & 0b00001000) >> 3;
	xbox.btnXbox = (btnBits & 0b00010000) >> 4;
	xbox.btnLS = (btnBits & 0b00100000) >> 5;
	xbox.btnRS = (btnBits & 0b01000000) >> 6;
	btnBits = rx_buf[XBOX_CONTROLLER_INDEX_BUTTONS_SHARE];
	xbox.btnShare = btnBits & 0b00000001;

	btnBits = rx_buf[XBOX_CONTROLLER_INDEX_BUTTONS_DIR];
	xbox.btnDirUp = btnBits == 1 || btnBits == 2 || btnBits == 8;
	xbox.btnDirRight = 2 <= btnBits && btnBits <= 4;
	xbox.btnDirDown = 4 <= btnBits && btnBits <= 6;
	xbox.btnDirLeft = 6 <= btnBits && btnBits <= 8;

	uint16_t joybuf[4];
	joybuf[0] = (uint16_t)rx_buf[0] | ((uint16_t)rx_buf[1] << 8);
	joybuf[1] = (uint16_t)rx_buf[2] | ((uint16_t)rx_buf[3] << 8);
	joybuf[2] = (uint16_t)rx_buf[4] | ((uint16_t)rx_buf[5] << 8);
	joybuf[3] = (uint16_t)rx_buf[6] | ((uint16_t)rx_buf[7] << 8);
	if (joybuf[0] > 65533){
		joybuf[0] = 65533;
	}

	if (joybuf[1] > 65533){
		joybuf[1] = 65533;
	}

	if (joybuf[2] > 65533){
		joybuf[2] = 65533;
	}

	if (joybuf[3] > 65533){
		joybuf[3] = 65533;
	}

	xbox.joyLHori = (int16_t)(joybuf[0] - maxJoy_dev2);
	xbox.joyLVert = -(int16_t)(joybuf[1] - maxJoy_dev2);
	xbox.joyRHori = (int16_t)(joybuf[2] - maxJoy_dev2);
	xbox.joyRVert = -(int16_t)(joybuf[3] - maxJoy_dev2);

	if (xbox.joyLHori <= 5 && xbox.joyLHori >= -5){
		xbox.joyLHori = 0;
	}

	if (xbox.joyLVert <= 5 && xbox.joyLVert >= -5){
		xbox.joyLVert = 0;
	}

	if (xbox.joyRHori <= 5 && xbox.joyRHori >= -5){
		xbox.joyRHori = 0;
	}

	if (xbox.joyRVert <= 5 && xbox.joyRVert >= -5){
		xbox.joyRVert = 0;
	}

	xbox.trigLT = (uint16_t)rx_buf[8] | ((uint16_t)rx_buf[9] << 8);
	xbox.trigRT = (uint16_t)rx_buf[10] | ((uint16_t)rx_buf[11] << 8);

	if (xbox.btnSelect_last && (!xbox.btnSelect)){
		xbox.btnSelect_falling = 1;
	}
	if (xbox.btnStart_last && (!xbox.btnStart)){
		xbox.btnStart_falling = 1;
	}
	if (xbox.btnShare_last && (!xbox.btnShare)){
		xbox.btnShare_falling = 1;
	}
	if (xbox.btnXbox_last && (!xbox.btnXbox)){
		xbox.btnXbox_falling = 1;
	}

	// 更新上次状态
	xbox.btnSelect_last = xbox.btnSelect;
	xbox.btnStart_last = xbox.btnStart;
	xbox.btnShare_last = xbox.btnShare;
	xbox.btnXbox_last = xbox.btnXbox;
}
