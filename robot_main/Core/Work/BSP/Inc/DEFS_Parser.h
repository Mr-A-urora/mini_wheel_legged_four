/*
	DEFS_Parser.h
	Copyright (c) JUST_HLL. All rights reserved.

	'Data Exchange Frame String Parser' struct
	by hjm
*/

#ifndef __DEFS_PARSER_H__
#define __DEFS_PARSER_H__

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#define DEFS_MAX_BUF_SIZE 64

#define INIT_PARSER(a,b) {(a).Init = DEFS_Init;\
						  (a).Reset = DEFS_Reset;\
						  (a).SetRawBuf = DEFS_SetRawBuf;\
						  (a).PushChar2RawBuf = DEFS_PushChar2RawBuf;\
						  (a).Verify = DEFS_Verify;\
						  (a).Parse = DEFS_Parse;\
						  (a).ClearPayload = DEFS_ClearPayload;\
						  (a).ParseSuccessHook = (b); }

typedef struct
{
	char cmd[8];
	char path[DEFS_MAX_BUF_SIZE];
	char value[16];
}Payload_t;

typedef struct _DEFS_Parser_t
{
	void(*Init)(struct _DEFS_Parser_t*, char char_header, char _is_report);
	void(*Reset)(struct _DEFS_Parser_t* self);
	unsigned char(*SetRawBuf)(struct _DEFS_Parser_t* self, const char* data_buf);
	unsigned char(*PushChar2RawBuf)(struct _DEFS_Parser_t* self, char ch);
	unsigned char(*Verify)(struct _DEFS_Parser_t* self);
	unsigned char(*Parse)(struct _DEFS_Parser_t* self);
	void(*ClearPayload)(struct _DEFS_Parser_t* self);
	void(*ParseSuccessHook)(void);

	unsigned short cur_raw_buf_length;
	unsigned char parse_done_flag;
	unsigned char parse_success_flag;

	char header;
	char is_report;
	float parsed_value;
	Payload_t payload;

	char buf_raw[DEFS_MAX_BUF_SIZE];
	char buf_temp[DEFS_MAX_BUF_SIZE];
	char buf_output[DEFS_MAX_BUF_SIZE];
}DEFS_Parser_t;

extern DEFS_Parser_t parser;

void DEFS_Init(DEFS_Parser_t* self, char char_header, char _is_report);

void DEFS_Reset(DEFS_Parser_t* self);

unsigned char DEFS_SetRawBuf(DEFS_Parser_t* self, const char* data_buf);

unsigned char DEFS_PushChar2RawBuf(DEFS_Parser_t* self, char ch);

unsigned char DEFS_Verify(DEFS_Parser_t* self);

unsigned char DEFS_Parse(DEFS_Parser_t* self);

void DEFS_ClearPayload(DEFS_Parser_t* self);




char MatchThenSet_FLOAT(DEFS_Parser_t* parser, const char* path, float* val, void(*SetSuccessHook)(void));

char MatchThenSet_U8(DEFS_Parser_t* parser, const char* path, unsigned char* val, void(*SetSuccessHook)(void));

char MatchThenSet_U16(DEFS_Parser_t* parser, const char* path, unsigned short* val, void(*SetSuccessHook)(void));

char MatchThenSet_SHORT(DEFS_Parser_t* parser, const char* path, short* val, void(*SetSuccessHook)(void));

#endif
