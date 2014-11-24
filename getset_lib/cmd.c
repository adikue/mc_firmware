#include "string.h"
#define __CMD_C
#include "cmd.h"
#include "getset.h"
#include <stdint.h>
/*------------extern section-----------------*/
/*------------extern section-----------------*/

/*------------proto section------------------*/
/*------------proto section------------------*/

/*-----------module config section-----------*/
#define COMMAND_LENGTH 128 // Maximum size of input;
command_t commands[] =\
{                         \
    {"GET", cmd_param_get},
    {"SET", cmd_param_set},
    {"SEND", cmd_send},
    {"CLEAR_TIMER", cmd_clear_timer},
    {"START_AT", cmd_start_at},
    {0,0}//End of list
};                                  
/*-----------module config section-----------*/

/*------------variables section--------------*/
int buffer_cur=0;
int buffer_arg_start=0;
int parser_state = 0;
char cmd_buffer[COMMAND_LENGTH+1];
/*------------variables section--------------*/




/*------------macros section-----------------*/
#define find_item(list,item,item_length,index)\
    index=0;\
    while(list[index].name!=0)\
    {\
        if((strlen(list[index].name)==(item_length))&&(strncmp(list[index].name,item,item_length)==0))\
            break;\
        index++;\
    }\
    index = list[index].name!=0 ? index : -1;
/*------------macros section-----------------*/

/*-----------module code section-----------*/
                      
int run_cmd(command_t* cmd_list,char* command_name)
{
    int i=0;
    find_item(commands,command_name,(buffer_arg_start-1),i);
    if (i==-1) 
        return 1;
    cmd_list[i].func(cmd_buffer+buffer_arg_start);
    return 0;
}


void cmd_buffer_add(char c)
{
    if(buffer_cur<COMMAND_LENGTH)
    {
        cmd_buffer[buffer_cur]=c;
        buffer_cur++;
    }
}
void cmd_parse(unsigned char rx_byte)
{
    if(rx_byte <32 && buffer_cur > 0)
    {
        if(buffer_arg_start ==0)
            buffer_arg_start = buffer_cur+1;
        cmd_buffer[buffer_cur]= 0;
        run_cmd(commands,cmd_buffer);
        buffer_cur =0;
        parser_state = 0;
    }
    else
    {
        cmd_buffer_add(rx_byte);
        switch(parser_state)
        {
            //Initial
            case 0:
                buffer_cur = 0;
                buffer_arg_start = 0;
                cmd_buffer_add(rx_byte);
                parser_state = 1; 
                //To skip first spaces
                //No brake - go to first state immidiately
            case 1:
                if(rx_byte == ' ')
                {
                    buffer_cur --;
                    break;
                }
                else
                    parser_state= 2;
            break;
            case 2:
                if(rx_byte == ' ')
                {
                    buffer_arg_start = buffer_cur;  
                    parser_state = 3;
                }
            break;
            case 3:
            break;
        }
    }
}

/*-----------module code section-----------*/






