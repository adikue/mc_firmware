#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include "config_variables_types.h" //Some function works with 

typedef enum{RS232 = 0, RS485 = 1, RS_ERROR = -1} rs_t;
typedef enum{SLAVE = 0, MASTER = 1} ms_t;

extern uint8_t rs_started;
extern uint8_t spi_started;
/*-------------------------------------------------------------*/
/* Initialization section */

void init(void);
static void clock_init(void);//should not be used from outside. Use init()
static void gpio_init(void);//should not be used from outside. Use init()
static void timers_init(void);


void SystemInit(void);

void conf_periph_init(void);
static void conf_clock_init(void);//should not be used from outside. Use init()
static void conf_gpio_init(void);//should not be used from outside. Use init()
static void conf_timers_init(void);//should not be used from outside. Use init()

/* END Initialization section */
/*-------------------------------------------------------------*/
/* USART section */
#define send_to_host(_str) USART1_put_str(_str);

//Declare USART1 functions from usart.h
extern int USART1_put_str(char*);
extern char USART1_take_char(void);
extern void USART6_init(void);
extern void USART6_stop(void);
extern int USART6_put_str(char* str);

/* END USART section */
/*-------------------------------------------------------------*/
/* Control section */

//SPI control functions
extern void spi_init(void);
extern void spi_stop(void);
extern void set_spi_voltage(spi_voltage_t);
extern int spi_select_cs(spi_channel_t);
extern void spi_deselect(void);
extern int spi_send_data(char* , uint32_t , spi_channel_t );
extern void spi_poll(void);

//RS GPIO control lines
void rs_set_fast(void);
void rs_clear_fast(void);
void rs_shutdown(void);
void rs_poweron(void);
rs_t rs_get_rs_mode(void);
void set_rs_mode(rs_t rs_mode);

//General control
ms_t get_ms_mode(void);
void reset_timers(void);
//Get timestamp
uint32_t get_timestamp(void);
//Reset timestamp counter
void timestamp_restart(void);
//Register start RX at <timestamp>
int srart_rx_at(uint32_t timestamp);
void clear_start_at(void);

/* END Control section */

//MISC
void delay_us(uint32_t us);

#endif /*HAL_H*/
