#include "spi.h"
#include "fifo.h"
#include "interface_processor.h"
#include "utils.h"


/* Configurable variables externs */
extern uint32_t SPI_DIV;
extern uint32_t SPI_MODE;
extern fixed_string_t POLL_DATA0;
extern fixed_string_t POLL_DATA1;
extern fixed_string_t POLL_DATA2;
extern spi_voltage_t SPI_OUT_VOLTAGE;
extern uint32_t COMMON_POLLING_ENABLED;
/* Configurable variables externs */

MAKE_FIFO(SPI_TX0, char, SPI_TX_BUFFER_SIZE)
MAKE_FIFO(SPI_TX1, char, SPI_TX_BUFFER_SIZE)
MAKE_FIFO(SPI_TX2, char, SPI_TX_BUFFER_SIZE)
volatile static uint8_t buffer0_is_busy = 0;
volatile static uint8_t buffer1_is_busy = 0;
volatile static uint8_t buffer2_is_busy = 0;
    
static spi_channel_t spi_ch_to_send = spi_ch0;
static uint8_t poll_started = 0;
uint8_t spi_state = spi_uninitilized;


uint16_t calc_br_reg(uint32_t _divider)
{
    if(_divider == 2) 
        return 0;
    if(_divider == 4) 
        return SPI_CR1_BR_0;
    if(_divider == 8) 
        return SPI_CR1_BR_1;
    if(_divider == 16) 
        return SPI_CR1_BR_0|SPI_CR1_BR_1;
    if(_divider == 32) 
        return SPI_CR1_BR_2;
    if(_divider == 64) 
        return SPI_CR1_BR_2|SPI_CR1_BR_0;
    if(_divider == 128) 
        return SPI_CR1_BR_2|SPI_CR1_BR_1;
    if(_divider == 256) 
        return SPI_CR1_BR_2|SPI_CR1_BR_1|SPI_CR1_BR_0;
    
    return 0;//if num incorrect set divider = 2
}   

/**
  * @brief  Initializes the SPIx peripheral.
  * @retval None
  */
void spi_init(void)
{
    uint16_t cpol = 0, cpha = 0;
    
    NVIC_DisableIRQ(SPI2_IRQn);
    
    SPI2->CR1 = SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_LSBFIRST;//Set master and LSB first. LSB needs to be discussed.
    SPI2->CR1 |= calc_br_reg(SPI_DIV);//Set SPI SCLK frquency divider of APB1 bus clock (24MHz)
    //Write SPI mode
    if (SPI_MODE > 1)
        cpol = SPI_CR1_CPOL;
    if (SPI_MODE == 1 || SPI_MODE == 3)
        cpha = SPI_CR1_CPHA;
    SPI2->CR1 |= cpol|cpha;
    NVIC_DisableIRQ(SPI2_IRQn);
    SPI2->CR2 = SPI_CR2_TXEIE;
    SPI2->CR2 |= SPI_CR2_RXNEIE;//Start reception and Enable reception interrupt
    spi_state |= spi_initialized;
    SPI2->CR1 |= SPI_CR1_SPE;
    NVIC_EnableIRQ(SPI2_IRQn);
    spi_state |= spi_rx_started;//Update state
}

void spi_stop(void)
{
    if ( !(spi_state & spi_rx_started) )//If spi is not running
        return;
        
    if (COMMON_POLLING_ENABLED){
        //stop polling sending and sync line swap
        NVIC_DisableIRQ(TIM2_IRQn);
    }
    
    //disable transmission interrupts
    while (SPI2->SR & (SPI_SR_BSY) ) {
        // Wait until send and receive end ( according 28.3.8  Disabling the SPI)
    }
    SPI2->CR1 &= ~SPI_CR1_SPE;// disable SPI. Enter Halt mode
    NVIC_DisableIRQ(SPI2_IRQn);
    spi_deselect(); 
        
    //call interface_finalize() to send received data if any
    interface_finalize();
    
    //at this point interface is stopped and can be reconfigured
    spi_state &= ~spi_rx_started;
}

void SPI2_IRQHandler(void)
{  
    volatile unsigned int IIR;
    IIR = SPI2->SR;
    if ( (IIR & SPI_SR_BSY) )// read interrupt
        return;
    
    if ( (IIR & SPI_SR_RXNE) ) {// read interrupt
        uint16_t byte = SPI2->DR;
        intf_add_byte(byte);
        if (!poll_started){
            interface_finalize();//Call to send collected data
        }
    }
    
    if  (IIR & SPI_SR_TXE) {//if transmit buffer is empty
        char buf_el = 0;
        if(spi_ch_to_send == spi_ch0){      
            if (!SPI_TX0_isempty()) {//If there is something to send
                poll_started = 1;
                spi_select_cs(spi_ch0);//Select 0 channel in which we will send&recieve data
                buf_el = SPI_TX0_pop();//Take TX buffer element
                SPI2->DR = buf_el;//Write into SPI bus
            }else{//If last packet byte has been sent in previous call then last answer byte has been recieved in this call. 
                //Or there was nothing to send to this channel
                spi_ch_to_send = spi_ch1;//Process next channel buffer
                buffer0_is_busy = 0;
                spi_deselect();//Deselect from previous transfer
            }
        }
                
        if(spi_ch_to_send == spi_ch1){          
            if (!SPI_TX1_isempty()) {//If there is something to send
                poll_started = 1;
                spi_select_cs(spi_ch1);//Select 1 channel in which we will send&recieve data
                buf_el = SPI_TX1_pop();//Take TX buffer element
                SPI2->DR = buf_el;//Write into SPI bus
            }else{//If last packet byte has been sent in previous call then last answer byte has been recieved in this call
                //Or there was nothing to send to this channel
                spi_ch_to_send = spi_ch2;//Process next channel buffer
                buffer1_is_busy = 0;
                spi_deselect();//Deselect from previous transfer
            }
        }       
        
        if(spi_ch_to_send == spi_ch2){  
            if (!SPI_TX2_isempty()) {//If there is something to send
                poll_started = 1;
                spi_select_cs(spi_ch2);//Select 2 channel in which we will send&recieve data
                buf_el = SPI_TX2_pop();//Take TX buffer element
                SPI2->DR = buf_el;//Write into SPI bus
            }else{//If last packet byte has been sent in previous call then last answer byte has been recieved in this call
                //Or there was nothing to send to this channel
                spi_ch_to_send = spi_ch0;//Process next channel buffer
                buffer2_is_busy = 0;
                spi_deselect();//Deselect from previous transfer
            }
        }

        if (!buffer0_is_busy && !buffer1_is_busy && !buffer2_is_busy){//If all channels buffers are empty
            SPI2->CR2 &= ~SPI_CR2_TXEIE;// disable TXEmpty interrupt if nothing to send
            poll_started = 0;//poll ended. All packets are sent
            interface_finalize();//Call to send collected data
        }
    }
}

/**
  * @brief  Add data to transmit buffer for SPI bus

  * @param  data: Pointer to data to be transmitted.

    * @param  data_len: Number of bytes to send from data

    * @param  ch_num: Number of channel for data transmission

  * @retval Status. The returned value can be one  of the follewong
    *                        0: success
    *                       -1: channel nummer is wrong
  */
int spi_send_data(char* data, uint32_t data_len, spi_channel_t ch_num)
{   
    uint16_t i;

    switch(ch_num){
        case spi_ch0:
            if (buffer0_is_busy == 0){
                for (i = 0; i < data_len; i++){ 
                    SPI_TX0_push(data[i]);
                }
            }
            buffer0_is_busy = 1;
            break;
            
        case spi_ch1:
            if (buffer1_is_busy == 0){
                for (i = 0; i < data_len; i++){ 
                    SPI_TX1_push(data[i]);
                }
            }
            buffer1_is_busy = 1;
            break;
            
        case spi_ch2:
            if (buffer2_is_busy == 0){
                for (i = 0; i < data_len; i++){ 
                    SPI_TX2_push(data[i]);
                }
            }
            buffer2_is_busy = 1;
            break;
            
        default:
            return -1;
    }
    
    SPI2->CR2 |= SPI_CR2_TXEIE;// enable TX interrupt
    
  return 0;
}

void spi_poll(void)
{ /* INTERRUPT CONTEXT CALL*/
    if (!poll_started){
        spi_send_data(POLL_DATA0.data, POLL_DATA0.length, spi_ch0);
        spi_send_data(POLL_DATA1.data, POLL_DATA1.length, spi_ch1);
        spi_send_data(POLL_DATA2.data, POLL_DATA2.length, spi_ch2);
    }
}

void set_spi_voltage(spi_voltage_t voltage)
    {
        //power selector line - PA7
        switch (voltage){
            case spi_out_3v:
                GPIOA->ODR &= !GPIO_ODR_ODR_7;//Set spi power selector to 3.3V
                break;
            case spi_out_5v:
                GPIOA->ODR |= GPIO_ODR_ODR_7;//Set spi power selector to 5V
                break;              
        }
        return;
    }

int spi_select_cs(spi_channel_t ch_num)
{
    switch (ch_num){
        case spi_ch0:
            //CS0 line - PB7
            GPIOB->ODR &= ~GPIO_ODR_ODR_7;//Set CS0 line to LOW
            break;
        case spi_ch1:
            //CS1 line - PB6
            GPIOB->ODR &= ~GPIO_ODR_ODR_6;//Set CS1 line to LOW
            break;
        case spi_ch2:
            //CS2 line - PB5
            GPIOB->ODR &= ~GPIO_ODR_ODR_5;//Set CS2 line to LOW
            break;
        default:
            return -1;  
    }
    return 0;
}

void spi_deselect(void)
{
    //CS0 line - PB5
    GPIOB->ODR |= GPIO_ODR_ODR_5;//Set CS2 line to HIGH
    //CS1 line - PB6
    GPIOB->ODR |= GPIO_ODR_ODR_6;//Set CS1 line to HIGH
    //CS2 line - PB7
    GPIOB->ODR |= GPIO_ODR_ODR_7;//Set CS0 line to HIGH
}
