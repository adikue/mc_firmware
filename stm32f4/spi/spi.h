#ifndef SPI_H
#define SPI_H
#include "stm32f4xx.h"
#include "config_variables_types.h"

//SPI states
#define spi_uninitilized                0x0
#define spi_initialized                 0x1
#define spi_rx_started                  0x2
#define spi_rx_wait_for_timer   0x4

/**
    * @brief  Initializes the SPI peripheral.
    
    * @retval None
*/
void spi_init(void);

void spi_stop(void);

/**
    * @brief  Add data to transmit buffer for SPI bus   

    * @param  data: Pointer to data to be transmitted.  

    * @param  data_len: Number of bytes to send from data 

    * @param  ch_num: Number of channel for data transmission   

    * @retval Status. The returned value can be one  of the follewong
    *                        0: success
    *                       -1: channel nummer is wrong
    *                   -2: data contains less then data_len bytes
*/
int spi_send_data(char* data, uint32_t data_len, spi_channel_t ch_num);

void spi_poll(void);

void set_spi_voltage(spi_voltage_t voltage);
int spi_select_cs(spi_channel_t ch_num);
spi_channel_t spi_selected_ch(void);
void spi_deselect(void);

#endif
