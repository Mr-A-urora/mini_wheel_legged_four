#include "DEFS_Parser.h"
DEFS_Parser_t parser;

void DEFS_Init(DEFS_Parser_t* self, char char_header, char _is_report)
{
	self->header = char_header;
	self->is_report = _is_report;
	self->parsed_value = 0.0f;
	self->Reset(self);
}


void DEFS_Reset(DEFS_Parser_t* self)
{
	self->cur_raw_buf_length = 0;
	self->parse_done_flag = 0;
	self->parse_success_flag = 0;

	memset(self->buf_raw, '\0', DEFS_MAX_BUF_SIZE);
	memset(self->buf_temp, '\0', DEFS_MAX_BUF_SIZE);
	memset(self->buf_output, '\0', DEFS_MAX_BUF_SIZE);

	self->ClearPayload(self);
}


unsigned char DEFS_SetRawBuf(DEFS_Parser_t* self, const char* data_buf)
{
	unsigned short temp_len = strlen(data_buf);
	if (!temp_len) 
		return 0;
	self->cur_raw_buf_length = temp_len;
	memcpy(self->buf_raw, data_buf, self->cur_raw_buf_length);
	return 1;
}


unsigned char DEFS_PushChar2RawBuf(DEFS_Parser_t* self, char ch)
{
	static unsigned char begin_new_frame_flag = 0;
	static unsigned short recv_index = 0;

	if (ch == self->header)
		begin_new_frame_flag = 1;

	if (begin_new_frame_flag == 1)
	{
		if (recv_index >= DEFS_MAX_BUF_SIZE)
		{
			begin_new_frame_flag = 0;
			recv_index = 0;
			self->Reset(self);
			return 0;
		}
		self->buf_raw[recv_index] = ch;
		recv_index++;
	}

	if (self->buf_raw[recv_index - 2] == '\r' && self->buf_raw[recv_index - 1] == '\n')
	{
		self->cur_raw_buf_length = recv_index;
		begin_new_frame_flag = 0;
		recv_index = 0;
		if (self->is_report)
			printf("[DEFS] data ready!\r\n");
		return 1;
	}
	else
	{
		return 0;
	}
}


unsigned char DEFS_Verify(DEFS_Parser_t* self)
{
	char* p = self->buf_raw;
	unsigned int sum = 0;
	char volatile suffix;
	while (*p != '\0')
	{ 
		if (*(p + 1) == '\r' && *(p + 2) == '\n')
			break;
		sum += *p;
		p++;
	}
	suffix = 'A' + sum % 26;
	
	return 1;

//	if (*p == suffix)
//	{
//		if (self->is_report)
//			printf("[DEFS] verify passed!\r\n");
//		return 1;
//	}
//	else
//	{
//		if (self->is_report)
//			printf("[DEFS] verify didn't pass!\r\n");
//		return 0;
//	}
}


unsigned char DEFS_Parse(DEFS_Parser_t* self)
{
	int i;
	unsigned char index_cnt = 0;
	unsigned char space_index[2] = {0, 0};

	for (i = 0; i < self->cur_raw_buf_length; i++)
	{
		if (self->buf_raw[i] == ' ')
		{
			if(index_cnt < 2)
				space_index[index_cnt] = i;
			index_cnt++;
		}
	}

	if (index_cnt != 2)
	{ 
		if (self->is_report)
			printf("[DEFS] parse failed!\r\n");
		return 0;
	}
		

	memcpy(self->payload.cmd, self->buf_raw + 1, space_index[0] - 1);
	memcpy(self->payload.path, self->buf_raw + space_index[0] + 1, space_index[1] - space_index[0] - 1);
	memcpy(self->payload.value, self->buf_raw + space_index[1] + 1, self->cur_raw_buf_length - space_index[1] - 4);

	if (self->is_report)
		printf("[DEFS] parse successed!\r\n");
	if (self->ParseSuccessHook != NULL)
		self->ParseSuccessHook();

	return 1;
}


void DEFS_ClearPayload(DEFS_Parser_t* self)
{
	memset(self->payload.cmd, '\0', 8);
	memset(self->payload.path, '\0', DEFS_MAX_BUF_SIZE);
	memset(self->payload.value, '\0', 16);
}


char MatchThenSet_FLOAT(DEFS_Parser_t* parser, const char* path, float* val, void(*SetSuccessHook)(void))
{
	if (strcmp(parser->payload.path, path) == 0)
	{
		*val = parser->parsed_value;
		if(SetSuccessHook != NULL)
			SetSuccessHook();
		return 1;
	}
	return 0;
}


char MatchThenSet_U8(DEFS_Parser_t* parser, const char* path, unsigned char* val, void(*SetSuccessHook)(void))
{
	if (strcmp(parser->payload.path, path) == 0)
	{
		*val = (unsigned char)parser->parsed_value;
		if(SetSuccessHook != NULL)
			SetSuccessHook();
		return 1;
	}
	return 0;
}


char MatchThenSet_U16(DEFS_Parser_t* parser, const char* path, unsigned short* val, void(*SetSuccessHook)(void))
{
	if (strcmp(parser->payload.path, path) == 0)
	{
		*val = (unsigned short)parser->parsed_value;
		if(SetSuccessHook != NULL)
			SetSuccessHook();
		return 1;
	}
	return 0;
}



char MatchThenSet_SHORT(DEFS_Parser_t* parser, const char* path, short* val, void(*SetSuccessHook)(void))
{
	if (strcmp(parser->payload.path, path) == 0)
	{
		*val = (short)parser->parsed_value;
		if(SetSuccessHook != NULL)
			SetSuccessHook();
		return 1;
	}
	return 0;
}
