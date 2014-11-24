#include "usart.h"
#include "fifo.h"
#include "interface_processor.h"

extern uint32_t PCLK2_Frequency;
uint8_t rs_state = rs_uninitilized;

uint32_t calc_stopbit_reg(double _num_stopbits)
{
    if(_num_stopbits == 1.0) 
        return 0;
    if(_num_stopbits == 0.5) 
        return 1;
    if(_num_stopbits == 1.5) 
        return 3;
    if(_num_stopbits == 2.0) 
        return 2;
    
    return 0;//if num incorrect set 1 stopbit
}               

/*------------USART1 section------------*/

MAKE_FIFO(USART1_TX, char, COMP_USART_TX_BUF_SIZE)
MAKE_FIFO(USART1_RX, char, COMP_USART_RX_BUF_SIZE)
volatile static char tx1_restart = 1;

void USART1_init(void){
    int i, div_mantissa, div_fraction;
    double usartdiv;
    
    usartdiv = (double)(PCLK2_Frequency)/(double)(COMP_BAUDRATE*16);//921600 by default. look into config.h
    div_mantissa = (int)usartdiv;
    div_fraction = (int)(((usartdiv - (double)div_mantissa)*16.0) + 0.5);
    
    USART1->BRR = (div_mantissa<<4)|div_fraction;
    USART1->CR1 = USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_TXEIE|USART_CR1_RXNEIE;
    USART1->CR2 = calc_stopbit_reg(COMP_STOPBITS) << 12;// 2 stop bits by default. look into config.h
    USART1->CR3 = 0x0000;// no flow control
    for (i = 0; i < 0x1000; i++) __NOP();// avoid unwanted output
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1  |= ((   1UL << 13) );// enable USART
}

void USART1_IRQHandler (void) {
  volatile unsigned int IIR;
    IIR = USART1->SR;
    if (IIR & USART_SR_RXNE) {// read interrupt
        if(!USART1_RX_isfull())
            USART1_RX_push(USART1->DR & 0xFF);
        else
            USART1_put_str("##\0");//Signaling that byte was dropped
        return;
    }
    if (IIR & USART_SR_TXE) {
        if (!USART1_TX_isempty()) {
            USART1->DR = (uint16_t)USART1_TX_pop();
            tx1_restart = 0;
        }
        else {
            tx1_restart = 1;
            USART1->CR1 &= ~USART_CR1_TXEIE;// disable TX interrupt if nothing to send
        }
    }
}

int USART1_PutChar(char c)
{
  if (USART1_TX_isfull())
    return (-1);
  USART1_TX_push(c);
  if (tx1_restart) {//If transmit interrupt is disabled, enable it
    tx1_restart = 0;
        USART1->CR1 |= USART_CR1_TXEIE;// enable TX interrupt
  }
  return 0;
}

int USART1_put_str(char* str)
{
    while(*str!=0)
    {
        USART1_PutChar(*str);
        str++;
    }
    return 0;
}

char USART1_take_char(void)
{
    if(!USART1_RX_isempty()){//If buffer is not empty
        return USART1_RX_pop();
    }
    return 0;
}