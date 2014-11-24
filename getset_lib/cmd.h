#ifndef _CMD_H
#define _CMD_H

struct _command_t {const char* name; void(*func)(char*);} typedef command_t;

void cmd_parse(unsigned char c);

#endif
