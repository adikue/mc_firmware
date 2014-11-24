#include "getset_internal.h"
#include "utils.h"
#include "config_variables.h" //Add configurable variables
#include "hal.h"

/* Set callbacks */

int change_variable(void *p, values_t variance, var_bound vlue_bound, uint32_t prev_val)
{
    int set_failed = 1;
    int i;
    
#if defined (RS_ENABLED)
    USART6_stop();
#elif defined (SPI_ENABLED)
    spi_stop();
#endif
    
    //Check variables for validity
    if (variance == discrete){
        if (vlue_bound.boundary != NULL){
            for (i = 0; i < vlue_bound.num_el; i++){
                if (vlue_bound.boundary[i] == *(uint32_t*)p)
                    break;
            }
            if (i < vlue_bound.num_el)  
                set_failed = 0;
        }
    }
    if (variance == min_max){
        if (vlue_bound.boundary != NULL){
            if( *(uint32_t*)p >= vlue_bound.boundary[0] && *(uint32_t*)p <= vlue_bound.boundary[1])
                set_failed = 0;
        }
    }
    
    if (variance == any)
        set_failed = 0;
    
    if (set_failed){
        //restore previous value
        *(uint32_t*)p = prev_val;
        conf_periph_init();
        return 1;
    }
        
    //call interfaces init with new config
    conf_periph_init();
    
    return 0;
}

//Callback for variables change of which clear start_at command effect
int clear_start_at_callback(void *p, values_t variance, var_bound vlue_bound, uint32_t prev_val)
{
    int err = change_variable( p, variance, vlue_bound, prev_val);
    if (err == 0){
        clear_start_at();
        conf_periph_init();
    }
    return err;
}

//Callback for DATAX variables
int datax_change( void *p, values_t variance, var_bound vlue_bound, uint32_t prev_val)
{
    int err = change_variable(p, variance, vlue_bound, prev_val);
    if (err == 0){        
        //For SPI recalculate T_POLL minimum value. Because on SPI recieved data >= sent data.
        //That's why we cannot poll spi more frquently than time we can send all polling data.
        #if defined (SPI_ENABLED)
            t_poll_values[0] = (uint32_t)((POLL_DATA0.length + POLL_DATA1.length + POLL_DATA2.length
                                        + TIMESTAMP_SIZE + CRC_SIZE + 1) * 4 * USEC / (3 * COMP_SPEED_EFF));
            //Refresh T_POLL value
            if (T_POLL < t_poll_values[0])
                T_POLL = t_poll_values[0];
        #endif
        //On RS interface recieved data is not connected with sent data generaly. That's why we cannot stand T_POLL minimum period.
        //Here is only one restriction. Polling perios can not be less than 1,5 byte reception time.
        
        
        conf_periph_init();
    }
    
    return err;
}
int rs_mode_callback(void *p, values_t variance, var_bound vlue_bound, uint32_t prev_val)
{
    int err;
    if (RS_MODE_CONTROL == RS_MODE_SOFT){
        err = change_variable(p, variance, vlue_bound, prev_val);
        if (err == 0)
            set_rs_mode((rs_t)RS_MODE);
        
        return err;
    }else
        return 1;
}

int rs_baudrate_callback(void *p, values_t variance, var_bound vlue_bound, uint32_t prev_val)
{
    int err = change_variable(p, variance, vlue_bound, prev_val);
    if (err == 0){
        //Recalc minimum byte timeout as time of 1,5 byte reception
        rs_byte_timeout_values[0] = (uint32_t)(TIMEOUT_COEFF*USEC/RS_BAUDRATE_DEFAULT);
        //Refresh RS_BYTE_TIMEOUT value
        if (RS_BYTE_TIMEOUT > rs_byte_timeout_values[0])
            RS_BYTE_TIMEOUT = rs_byte_timeout_values[0];
        
        //Recalc minimum valid polling interval
        t_poll_values[0] = (uint32_t)(RS_BYTE_TIMEOUT);
        //Refresh T_POLL value
        if (T_POLL < t_poll_values[0])
            T_POLL = t_poll_values[0];
        
        //Store new maximum for sync impulse length
        rs_sync_t_values[1] = T_POLL - 1;
        //Refresh RS_SYNC_T value
        if (RS_SYNC_T > rs_sync_t_values[1])
            RS_SYNC_T = rs_sync_t_values[1];
    }
    
    return err;
}
/* Set callbacks */

#define PARAMETER_UINT(name,var,callback,val_type,val_boud)             {name,  (void*)&var,  0, callback, val_type, val_boud} 
#define PARAMETER_CONST_UINT(name,var)                                  {name,  (void*)&var,  1, 0, any, 0} 
#define PARAMETER_CONST_CHAR(name,value)                                {name,  (void*)value, 2, 0, any, 0}
#define PARAMETER_BASE64_STRING(name,var,callback,val_type,val_boud)    {name,  (void*)&var,  3, callback, val_type, val_boud}
//callback is "int func(void*)"-like function. return value will be passed to host as answer to 'set' command


struct param_table_item param_table[]  =
{
    PARAMETER_CONST_CHAR("WHO_AM_I",WHO_AM_I),
    PARAMETER_CONST_UINT("TIMER", CURRENT_TIMESTAMP),//NOT SUPPORTED YET
    
    //Polling data config
    PARAMETER_BASE64_STRING("DATA0", POLL_DATA0, datax_change, any, 0),
    PARAMETER_BASE64_STRING("DATA1", POLL_DATA1, datax_change, any, 0),
    PARAMETER_BASE64_STRING("DATA2", POLL_DATA2, datax_change, any, 0),
    
    //Common interfaces config
    PARAMETER_UINT("POLLING", COMMON_POLLING_ENABLED, clear_start_at_callback, discrete, COMMON_POLLING_BOUND),
    PARAMETER_UINT("RX", RX_ENABLED, clear_start_at_callback, discrete, RX_ENABLED_BOUND),
    PARAMETER_UINT("T_POLL", T_POLL, change_variable, min_max, T_POLL_BOUND),
    
    //SPI config
    PARAMETER_UINT("SPI_DIV", SPI_DIV, change_variable, discrete, SPI_DIV_BOUND),
    PARAMETER_UINT("SPI_MODE", SPI_MODE, change_variable, discrete, SPI_MODE_BOUND),
    PARAMETER_UINT("SPI_CHANNEL", SPI_CHANNEL, change_variable, discrete, SPI_CHANNEL_BOUND),
    PARAMETER_UINT("OUT_VOLTAGE", SPI_OUT_VOLTAGE, change_variable, discrete, SPI_OUT_VOLTAGE_BOUND),
    
    //RS config
    PARAMETER_UINT("RS_SPEED", RS_BAUDRATE, rs_baudrate_callback, min_max, RS_BAUDRATE_BOUND),
    PARAMETER_UINT("RS_STOPBITS", RS_STOPBITS, change_variable, discrete, RS_STOPBITS_BOUND),
    PARAMETER_UINT("RS_PARITY", RS_PARITY, change_variable, discrete, RS_PARITY_BOUND),
	PARAMETER_UINT("RS_SYNC_MODE", RS_SYNC_MODE, change_variable, discrete, RS_SYNC_MODE_BOUND),
    PARAMETER_UINT("EXTSYNC", EXTSYNC, change_variable, discrete, EXT_SYNC_BOUND),
    PARAMETER_UINT("SYNC_T", RS_SYNC_T, change_variable, min_max, RS_SYNC_T_BOUND),
    PARAMETER_UINT("TIMEOUT", RS_BYTE_TIMEOUT, change_variable, min_max, RS_BYTE_TIMEOUT_BOUND),
    PARAMETER_UINT("RS_MODE", RS_MODE, change_variable, discrete, RS_MODE_BOUND),
    {0, 0, 0,   0, any, 0},        //end of list
};
