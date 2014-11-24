#ifndef GETSET_INTERNAL_H
#define GETSET_INTERNAL_H
#include <stdint.h>
#include <stddef.h>
#include "config_variables_types.h"

typedef enum{
    any = 0,
    discrete = 1,
    min_max = 2
} values_t; 

struct param_handlers_t{
    int(*get)(char*, char*, size_t, void*);
    int(*set) (char*, void*);
};

struct param_table_item{
    const char  *name;
    void        *param;
    int         param_type;
    int         (*set_callback)(void *, values_t , var_bound , uint32_t);
    values_t    values;
    var_bound   values_bound;
};

#define MAX_PARAMETER_LENGTH    32+1

#endif /*GETSET_INTERNAL_H*/
