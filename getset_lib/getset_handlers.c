#include <string.h>
#include <stdio.h>
#include "getset_internal.h"
#include "utils.h"

/*------------proto section------------------*/
int int_get(char *input,char *output, size_t size,void* param);
int int_set(char*,void*);
int str_get(char *input,char *output, size_t size,void* param);
/*------------proto section------------------*/


struct param_handlers_t param_handlers[]=
{
    {int_get,int_set},
    {int_get,0},
    {str_get,0},
    {fixed_string_get64,fixed_string_set64},
};


int int_get(char *input,char *output, size_t size,void* param)
{
    snprintf(output,size,"%d",*((int*)param));
    return 0;
}
int int_set(char*str,void*param)
{
    return sscanf(str,"%d",(int*)param)==1?0:-1;
}
int str_get(char* input,char *output, size_t size,void* param)
{
    strcpy(output, (char*) param);
    return 0;
}
