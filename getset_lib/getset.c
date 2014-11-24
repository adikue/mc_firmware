#include "getset.h"
#include <string.h>
#include <stdio.h>
#include "hal.h"
#include "getset_internal.h"
/*------------extern section-----------------*/
extern spi_channel_t SPI_CHANNEL;
/*------------extern section-----------------*/

/*------------proto section------------------*/
int int_get(char *input,char *output, size_t size,void* param);
int int_set(char*,void*);
int str_get(char *input,char *output, size_t size,void* param);
/*------------proto section------------------*/

/*------------variables section--------------*/
volatile int enable_names=1;

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
/*-----------type definition section---------*/

/*-----------type definition section---------*/

/*-----------module config section-----------*/
extern const struct param_table_item param_table[];
extern struct param_handlers_t param_handlers[];
/*-----------module config section-----------*/

/*-----------code section--------------------*/
void cmd_param_get(char* argname)
{
    char str[MAX_PARAMETER_LENGTH];
    int i=0;
    find_item(param_table,argname,strlen(argname),i);
    if(i!=-1)
    {
        unsigned int type = param_table[i].param_type;
        void* param       = param_table[i].param;
        param_handlers[type].get("",str,MAX_PARAMETER_LENGTH,param);
        if(enable_names)
        {
            send_to_host(argname);
            send_to_host("=");
        }
        send_to_host(str);
    }
    send_to_host("\n");
    return;
}
void cmd_param_set(char * arg)
{
    unsigned int value_position = 0;
    int i;
    int callback_result;
    char buf[16];
    int prev_val;
    void* param ;
    unsigned int type;
    values_t values_var;
    var_bound value_boundary;
    
    while(arg[value_position]>32&&arg[value_position]!=' ')
        value_position++;
    if(arg[value_position]!=' ')
    {
        //wrong command
        return;
    }
    find_item(param_table,arg,value_position,i);
    if(i==-1)
        return;
    type = param_table[i].param_type;
    param   = param_table[i].param;
    values_var = param_table[i].values;
    value_boundary = param_table[i].values_bound;
    value_position++;
    if(param_handlers[type].set == 0)
    {
        send_to_host("-1\n");
        return;
    }
    
    prev_val = *(int*)param;
    if(param_handlers[type].set(arg+value_position,param))
    {
        send_to_host("-1\n");
        return;
    }
    
    if(param_table[i].set_callback)
        callback_result = param_table[i].set_callback(param, values_var, value_boundary, prev_val);
    else
        callback_result = 0;
    snprintf(buf, 16, "%d\n", callback_result);
    send_to_host(buf);
}

void cmd_send(char *data)
{
    char buf[16] = "0";
    int errno;
    #if defined (RS_ENABLED)
        errno = USART6_put_str(data);
    #elif defined (SPI_ENABLED)
        errno = spi_send_data(data, strlen(data), SPI_CHANNEL);
    #endif
    snprintf(buf, 16, "%d\n", errno);
    send_to_host(buf);
    
    return;
}

void cmd_clear_timer(char *null)
{
    timestamp_restart();
    send_to_host("0\n");
}

void cmd_start_at(char *timestamp)
{
    uint32_t tsmp = 0;
    int err;
    char buf[16] = "0";
    
    err = sscanf(timestamp,"%d", &tsmp);
    if (err == 1){
        int result = srart_rx_at(tsmp);
        snprintf(buf, 16, "%d\n", result);
        send_to_host(buf);
        return;
    }
    send_to_host("1\n");
    return;
    
}

/*-----------code section--------------------*/
                                                   
