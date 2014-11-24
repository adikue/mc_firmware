#include "hal.h"

#include <stm32f4xx_rcc.h>// Need for clock initialization
#include "misc.h" //Need for NVIC works
#include "usart.h" //Usart header
#include "spi.h" //spi header
#include "config.h" //Initialization works with config
#include "utils.h"  //For fixed_string_t
#include <stdio.h> //For snprintf

uint8_t tim2_enabled = 0;
uint32_t SYSCLK_Frequency; //SYSCLK clock frequency expressed in Hz
uint32_t HCLK_Frequency;   //HCLK clock frequency expressed in Hz
uint32_t PCLK1_Frequency;  //PCLK1 clock frequency expressed in Hz
uint32_t PCLK2_Frequency;  //PCLK2 clock frequency expressed in Hz
uint8_t clock_initialized = 0; //For propper delay_us() operation
uint8_t extsync_level = 0;

/* Configurable variables externs */
extern uint32_t RX_ENABLED;
extern uint32_t T_POLL;
extern uint32_t RS_MODE;
extern uint32_t RS_SYNC_MODE;
extern uint32_t RS_SYNC_T;
extern uint32_t RS_BYTE_TIMEOUT;
extern uint32_t COMMON_POLLING_ENABLED;
extern spi_voltage_t SPI_OUT_VOLTAGE;
extern uint8_t rs_state;
extern uint8_t spi_state;
extern uint32_t EXTSYNC;
extern fixed_string_t POLL_DATA0;
/* Configurable variables externs */

static uint32_t tpolltim_period;
#if defined (RS_ENABLED)
    static uint32_t rs_sync_inactive;
#endif

void assert_param(int param)
{
    if(param)
        return;
    else
        while(1){}
}

void delay_us(uint32_t us)
{
    uint32_t i;
    uint32_t ticks;
    if (clock_initialized)
        ticks = (SYSCLK_Frequency/1000000) * us;
    else
        ticks = 8 * us;//if clock is not initialized then sysclk is assumed to be 8MHz
    
    for (i = 0; i < ticks; i++)
        __NOP();
        
}
                    
void SETAFR(GPIO_TypeDef *port, uint8_t pin, uint8_t afr_n)
{
    uint32_t AFR = 0;
    
    if (pin >= 8){//Use GPIOx_AFRH
        AFR = port->AFR[1];//Get current GPIOx_AFRH
        AFR &= ~(0x0000000F << (4*(pin-8))); //Set ZERO current AFR settings
        AFR |= ((uint32_t)(afr_n&0xF)) << (4*(pin-8));//Set afr_n in ARH word
        port->AFR[1] = AFR;//Write afr settings
    } else {
        AFR = port->AFR[0];//Get current GPIOx_AFRL
        AFR &= ~(0x0000000F << (4*(pin-8))); //Set ZERO current AFR settings
        AFR |= ((uint32_t)(afr_n&0xF)) << (4*pin);//Set afr_n in ARH word
        port->AFR[0] = AFR;//Write afr settings
    }
}

void init()
{
    __disable_irq();
    
    //Initilize system not configurable perepherial
    clock_init();
    gpio_init();
    USART1_init();
    timers_init();
    
    conf_periph_init();//Call periph_init to initialize interfaces, timers and others for interfaces functionality
    
    //Interrupt priority groups
    
    /* Highest group.
    * Timestamp counter and USART1 should be processed firstly
    */
    NVIC_SetPriority (TIM2_IRQn, 0); //Timestamp counter
    NVIC_SetPriority (USART1_IRQn, 0); //USB data send/recieve
    /* Mid group.
    * Next  external interfaces interrupt should be processed
    */
    NVIC_SetPriority (USART6_IRQn, 1); //RS interface data send/recieve
    NVIC_SetPriority (TIM6_DAC_IRQn, 1); //RS interface paket ending timer
    NVIC_SetPriority (SPI2_IRQn, 1); //SPI data send/recieve
    /* Lowest group.
    * External interrupts from PINS should be processe lastly
    */    
    NVIC_SetPriority (EXTI1_IRQn, 2); //rst_in line
    NVIC_SetPriority (EXTI2_IRQn, 2); //RS ext_sync line and RS232/485 line
 
    __enable_irq();
}

void SystemInit(void)
{}

void conf_periph_init(void)
{
    __disable_irq();
    
    conf_clock_init();
    conf_gpio_init();
    
#if defined (RS_ENABLED)
    USART6_init();
#elif defined (SPI_ENABLED)
    spi_init();
#endif
    
    conf_timers_init();

    __enable_irq();
}


static void clock_init()
{
    uint32_t timeout;
    RCC_ClocksTypeDef rcc_clocks;
    
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
    #endif
    RCC_PLLCmd(DISABLE);
    //Working at 48 MHz
    RCC_HSEConfig(RCC_HSE_ON);//Configure High Speed oscillator ON
    if(RCC_WaitForHSEStartUp() != SUCCESS)// Wait for HSE to start
    {
        while(1){}
    }
    //HSE oscillator clock selected as PLL clock entry
    //8 - PLLM: specifies 2MHz PLL VCO input clock
  //192 - PLLN: specifies 384MHz VCO output clock
  //8 - PLLP: specifies the 384/8 = 48MHz system clock
  //8 - PLLQ: specifies the 384/8 = 48MHz OTG FS, SDIO and RNG clocks
    RCC_PLLConfig( RCC_PLLSource_HSE, 8,192,8,8);
    RCC_PLLCmd(ENABLE);
    FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_1WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY)  != FLASH_ACR_LATENCY_1WS)
        ;
    //PLL selected as system clock source (SYSCLK). 48MHz
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    timeout = 10000;
    while(!(RCC_GetSYSCLKSource() == 0x08) && timeout)
    {
         timeout--;
    }
    if(!timeout)
    {
        while(1){}
        //TODO: harware fault handler
    }
    //AHB clock = SYSCLK. 48MHz
    RCC_HCLKConfig(RCC_HCLK_Div1);
    //APB1 clock = HCLK/2. 24MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);
    //APB2 clock = HCLK. 48MHz
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|
                           RCC_AHB1Periph_GPIOB|
                           RCC_AHB1Periph_GPIOC,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    //Store selected frequencies
    RCC_GetClocksFreq(&rcc_clocks);
    SYSCLK_Frequency = rcc_clocks.SYSCLK_Frequency; 
    HCLK_Frequency = rcc_clocks.HCLK_Frequency;   
    PCLK1_Frequency = rcc_clocks.PCLK1_Frequency;  
    PCLK2_Frequency = rcc_clocks.PCLK2_Frequency;
    clock_initialized  = 1;

    if (get_ms_mode() == MASTER){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    }
}

static void conf_clock_init(void)
{
#if defined (RS_ENABLED)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
#elif defined (SPI_ENABLED)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
#endif
    
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP|DBGMCU_APB1_FZ_DBG_TIM3_STOP|RCC_APB1Periph_TIM6;
}

static void gpio_init()
{
    /*USART1*/
    SETAFR(GPIOA,9,7);//tx1
    SETAFR(GPIOA,10,7);//rx1
    GPIOA->MODER |= GPIO_MODER_MODER10_1|GPIO_MODER_MODER9_1;//Set mode to AF
        
    /*LEDs*/
    //Power on led
    GPIOB->MODER |= GPIO_MODER_MODER9_0;//Set as output
    GPIOB->ODR |=  GPIO_ODR_ODR_9;//Power led on
    
    /*CONTROL_LOGIC*/
    //master/slave line. PB1
    //set as input
    GPIOB->MODER &= ~(GPIO_MODER_MODER1_0|GPIO_MODER_MODER1_1);
    
    if (get_ms_mode() == MASTER){
        //rst_gen line. PA2
        GPIOA->MODER |= GPIO_MODER_MODER2_0;//Set as output
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_1;//Set pullDOWN
        
        /*TIM3. clk_gen line*/
        SETAFR(GPIOA,6,2);//tim3ch1. PA6
        GPIOA->MODER |= GPIO_MODER_MODER6_1;////Set mode to AF
    }
    
    //rst_in line. PA1
    GPIOA->MODER &= ~(GPIO_MODER_MODER1_0|GPIO_MODER_MODER1_1);//set as input
    EXTI->IMR |= EXTI_IMR_MR1;//Unmask interrupt line 1
    EXTI->RTSR |= EXTI_RTSR_TR1;//Interrupt on line 1 set for rising edge
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PA;//Unmask end enable interrupt on PA1 for line 1
    NVIC_EnableIRQ(EXTI1_IRQn);
    
    /*TIM2. clk_in line*/
    SETAFR(GPIOA,0,1);//tim2etr. PA0
    GPIOA->MODER |= GPIO_MODER_MODER0_1;////Set mode to AF
}

static void conf_gpio_init(void)
{   
#if defined (RS_ENABLED)
    /*USART6*/
    SETAFR(GPIOC,6,8);//tx6
    SETAFR(GPIOC,7,8);//rx6
    GPIOC->MODER |= GPIO_MODER_MODER7_1|GPIO_MODER_MODER6_1;//Set mode to AF
    
    //max3161 /shdn line. PC0
    GPIOC->MODER |= GPIO_MODER_MODER0_0;//Set as output
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_0;//Set pullUP
    GPIOC->ODR |=  GPIO_ODR_ODR_0;//Set high level
    rs_poweron();//Power on RS transmission chip
    
    //max3161 fast line. PC1
    GPIOC->MODER |= GPIO_MODER_MODER1_0;//Set as output
    rs_set_fast();//Set fast mode by default
    
    //max3161 rs485/232 line. PC2
    if (RS_MODE_CONTROL == RS_MODE_HARD){
        GPIOC->MODER &= ~(GPIO_MODER_MODER2_0|GPIO_MODER_MODER2_1);//Set as input
        EXTI->IMR |= EXTI_IMR_MR2;//Unmask interrupt line 
        EXTI->RTSR |= EXTI_RTSR_TR2;//Interrupt on line 2 set for rising edge
        EXTI->FTSR |= EXTI_FTSR_TR2;//Interrupt on line 2 set for falling edge
        SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PC;//Unmask and enable interrupt on PC2 for line 2
        NVIC_EnableIRQ(EXTI2_IRQn);
    }
    if (RS_MODE_CONTROL == RS_MODE_SOFT){
        GPIOC->MODER |= GPIO_MODER_MODER2_0;//Set as output
    }
    
    //max3161 hdplx line. PC3
    GPIOC->MODER |= GPIO_MODER_MODER3_0;//Set as output
    //full duplex by default

    //RS sync line. PC8
    GPIOC->MODER |= GPIO_MODER_MODER8_0;//Set as output
    
    //RS ext_sync line. PD2
    if (EXTSYNC){
        GPIOD->MODER &= ~(GPIO_MODER_MODER2_0|GPIO_MODER_MODER2_1);//set as input
        EXTI->IMR |= EXTI_IMR_MR2;//Unmask interrupt line 2
        EXTI->RTSR |= EXTI_RTSR_TR2;//Interrupt on line 2 set for rising edge
        EXTI->FTSR |= EXTI_FTSR_TR2;//Interrupt on line 2 set for falling edge
        SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PD;//Unmask and enable interrupt on PD2 for line 2
        extsync_level = (GPIOD->IDR & GPIO_IDR_IDR_2) ? 1 : 0;//Store strtup ext_sync level
        NVIC_EnableIRQ(EXTI2_IRQn);
    }
    
    if (RS_SYNC_MODE == 1 || RS_SYNC_MODE == 5){//SYNC_MODE == b001 or SYNC_MODE == 0b101
        GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_0;//Set Pull Down
        GPIOC->PUPDR |= GPIO_PUPDR_PUPDR8_1;//Set Pull Down
        GPIOC->ODR &= ~GPIO_ODR_ODR_8;//Set Low
        rs_sync_inactive = 0;
    }else if (RS_SYNC_MODE == 3 || RS_SYNC_MODE == 7){//SYNC_MODE == b011 or SYNC_MODE == 0b111
        GPIOC->PUPDR |= GPIO_PUPDR_PUPDR8_0;//Set Pull Up
        GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_1;//Set Pull Up
        GPIOC->ODR |= GPIO_ODR_ODR_8;//Set High
        rs_sync_inactive = 1;
    }else
        rs_sync_inactive = 0;
    
#elif defined (SPI_ENABLED)
    /*SPI2*/
    SETAFR(GPIOB,13,5);//sck2
    SETAFR(GPIOB,14,5);//miso2
    SETAFR(GPIOB,15,5);//mosi2
    GPIOB->MODER |= GPIO_MODER_MODER13_1|GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1;//Set mode to AF
    
    //spi cs lines. PB5. PB6. PB7
    GPIOB->MODER |= GPIO_MODER_MODER5_0|GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0;//Set this outputs
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR5_0|GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR7_0;//Set pullUP
    spi_deselect();
#endif
    //spi voltage select line. PA7
    GPIOA->MODER |= GPIO_MODER_MODER7_0;//Set as output
    set_spi_voltage(SPI_OUT_VOLTAGE);//Set spi lines and rs sync line voltage
}


//Interupt handler for rst_in line
void EXTI1_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR1;//Reset interrupt penging flag
    /* In normal condition this interrupt triggers on rising edge on PA1 
    *   and TIM2 counter enabled */
    if(tim2_enabled){//If timer counts. Normal condition
        TIM2->CR1 &= ~TIM_CR1_CEN;//Stop counter
        TIM2->CNT = 0;//Reset counter
        tim2_enabled = 0;
        //Next triggers on falling edge
        EXTI->FTSR |= EXTI_FTSR_TR1;//Enable interrupt on falling edge
        //Clear "start_at" command effect
        clear_start_at();
        return;
    }else{
        TIM2->CR1 |= TIM_CR1_CEN;//Start counter
        //Next is normal condition. Triggers on rising edge
        EXTI->FTSR &= !EXTI_FTSR_TR1;//Disable interrupt on falling edge
        return;
    }
}

//Interupt handler for max3161 rs485/232 and ext_sync lines.
void EXTI2_IRQHandler(void)
{
    uint8_t pd2_lvl = (uint8_t)((GPIOD->IDR & GPIO_IDR_IDR_2) >> 2);
    EXTI->PR |= EXTI_PR_PR2;//Reset interrupt penging flag
    
	//Hanling rs485/232 line changing
    if (GPIOC->IDR & GPIO_IDR_IDR_2)//Line high level
        RS_MODE = RS485;
    else
        RS_MODE = RS232;
		
    //Hanling ext_sync line changing
    if ( extsync_level == 1 && pd2_lvl==0 ){//If falling edge interrrupt 
        if(EXTSYNC == 1){
            uint32_t tm = TIM2->CNT;
            char buf[16];
            buf[0] = '/';
            snprintf(buf+1,15,"%d",tm);
            send_to_host(buf);
        }
        if(EXTSYNC == 2){
            timestamp_restart();
            EXTSYNC = 0;//Reset EXT_SYNC mode to OFF
            SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PD;//Mask interrupt on PD2 for line 2
        }
    }
    extsync_level = pd2_lvl;//Store current ext_sync level
}

static void timers_init(void)
{
    /*TIM3. Used for generate clk_gen line on master*/
    if (get_ms_mode() == MASTER){
        //Output compare mode
        TIM3->PSC = (uint16_t)((PCLK1_Frequency/TIM3_FREQ) - 1);
        TIM3->ARR = TIM3_FREQ/CLK_GEN_FREQ - 1;
        TIM3->CCMR1 |=  TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1PE;// Swap pin level, preload enable
        TIM3->CCER |= TIM_CCER_CC1E;//Capture/Compare 1 output enable
        TIM3->EGR |= TIM_EGR_UG;
        TIM3->CR1 |= TIM_CR1_CEN;//Start count
    }
    
    /*TIM2. Used as timestamp counter. ETR mode*/
    //Now this timer count us.
    TIM2->SMCR &= ~TIM_SMCR_ETF;//No filter
    TIM2->SMCR &= ~TIM_SMCR_ETPS;//Prescaler = 0. External clock must be 1MHz
    TIM2->SMCR &= ~TIM_SMCR_ETP;//Rising edge detection
    TIM2->SMCR |= TIM_SMCR_ECE;//External clock mode2 (ETR)
    TIM2->CNT = 0;//Reset counter
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;//Start count
    tim2_enabled = 1;
}

static void conf_timers_init(void)
{   
#if defined (RS_ENABLED)        
    /*TIM6. Used for RS interface byte reception timeout*/  
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->PSC = (uint16_t)((2*PCLK1_Frequency/TIM6_FREQ) - 1); //1 tick per 1 us.
    TIM6->ARR = RS_BYTE_TIMEOUT - 1;
    TIM6->CR1 |= TIM_CR1_URS;//Only counter overflow generates an UEV
    //UEV enabled by edfault
    TIM6->DIER |= TIM_DIER_UIE;//Update interrupt enable
    TIM6->EGR |= TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;//Start count
    NVIC_EnableIRQ(TIM6_DAC_IRQn);  
#endif
    
    if (COMMON_POLLING_ENABLED){
        /*TIM2 CH2 is used to generate poll interrupt on SPI or RS*/
        TIM2->DIER |= TIM_DIER_CC2IE;
        TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;//Output CC2
        TIM2->CCER |= TIM_CCER_CC2E;
        TIM2->CCR2 = TIM2->CNT + T_POLL;
        tpolltim_period = T_POLL;
        NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void TIM2_IRQHandler (void) {
    uint32_t cnt_buf = TIM2->CNT;
    uint32_t start = TIM2->CNT;
    uint16_t IIR = TIM2->SR;
    //RS sync line. PC8
    uint32_t sync_line_state = (uint8_t)((GPIOC->ODR & GPIO_ODR_ODR_8) >> 8);//Get line state;
    
    if (IIR & TIM_SR_CC2IF){//CH2 used as poll timer
        TIM2->SR &= ~TIM_SR_CC2IF;// Clear timer interrupt status
        
#if defined(SPI_ENABLED)
        cnt_buf += tpolltim_period;////Trigger timer after polling period
        spi_poll();
#elif defined(RS_ENABLED)
        if (RS_SYNC_MODE >= 5){// If sync line need to be swapped
            uint32_t odr = GPIOC->ODR;
            odr &= ~GPIO_ODR_ODR_8; //Clear  PC8 state in buffer
            if (sync_line_state == rs_sync_inactive){ //If line in inactive state
                GPIOC->ODR = odr | ((!rs_sync_inactive)<<8);//Set sync line to active state
                cnt_buf += RS_SYNC_T;//Trigger timer after length of impulse
            }else{
                GPIOC->ODR = odr | (rs_sync_inactive<<8);//Set sync line to inactive state
                cnt_buf += tpolltim_period - RS_SYNC_T;//Trigger timer after period of impulse
            }
        }else{//If sync line is not swapping
            cnt_buf += tpolltim_period;////Trigger timer after polling period
            if (POLL_DATA0.length > 0)//If DATA0 is not empty
                USART6_put_str(POLL_DATA0.data);//Send polling data
        }
#endif
        
        TIM2->CCR2 = cnt_buf;//Store new trigger value
    }
        
    if (IIR & TIM_SR_CC3IF){//CH3 used as "START_AT" timer
        TIM2->SR &= ~TIM_SR_CC3IF;// Clear timer interrupt status
        TIM2->DIER &= ~TIM_DIER_CC3IE;//Disable this interrupt. It works once
        
        RX_ENABLED = 1;
        //Start interface
        #if defined (RS_ENABLED)
            USART6_init();
        #elif defined (SPI_ENABLED)
            spi_init();
        #endif  
    }
}

void timestamp_stop(void)
{
}

void timestamp_restart(void)
{
    uint32_t cr1_buf;
    __disable_irq();
    
    cr1_buf = TIM2->CR1;
    TIM2->CR1 &= ~TIM_CR1_CEN;//Stop count
    TIM2->CNT = 0;//Reset counter
    TIM2->CCR2 = 0; //Clears polling
    TIM2->CR1 = cr1_buf;//Restore CR1
    TIM2->EGR |= TIM_EGR_UG;//Update registers
    
    //Clear "start_at" command effect
    clear_start_at();
    //RX_ENABLED = 1; 
    
    __enable_irq();
}

int srart_rx_at(uint32_t timestamp)
{
    if (timestamp < TIM2->CNT + 5)//Correct only if timestamp  more than current + 5us
        return 1;
    
    RX_ENABLED = 0; 
    #if defined(SPI_ENABLED)
        spi_state |= spi_rx_wait_for_timer;
    #elif defined(RS_ENABLED)
        rs_state |= rs_rx_wait_for_timer;
    #endif
    /*TIM2 CH3 is used to generate RX start interrrupt*/
    NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->DIER |= TIM_DIER_CC3IE;
    TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;//Output CC2
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->CCR3 = timestamp;
    NVIC_EnableIRQ(TIM2_IRQn);
    return 0;
}

void clear_start_at(void)
{
    TIM2->DIER &= ~TIM_DIER_CC3IE;
    TIM2->CCR3 = 0;
    #if defined(SPI_ENABLED)
        spi_state &= ~spi_rx_wait_for_timer;
    #elif defined(RS_ENABLED)
        rs_state &= ~rs_rx_wait_for_timer;
    #endif
}

#if defined (RS_ENABLED)
    void rs_set_fast(void)
    {
        //rs fast line - PC1
        GPIOC->ODR |= GPIO_ODR_ODR_1;//Set rs fast line to HIGH
    }

    void rs_clear_fast(void)
    {
        //rs fast line - PC1
        GPIOC->ODR &= !GPIO_ODR_ODR_1;//Set rs fast line to LOW
    }
    void rs_shutdown(void)
    {
        //rs /shdn line - PC0
        GPIOC->ODR &= !GPIO_ODR_ODR_0;//Set rs /shdn line to LOW
    }

    void rs_poweron(void)
    {
        //rs /shdn line - PC0
        GPIOC->ODR = GPIO_ODR_ODR_0;//Set rs /shdn line to HIGH
    }
    
    void set_rs_mode(rs_t rs_mode)
    {
        //max3161 rs485/232 line. PC2
        if (RS_MODE_CONTROL == RS_MODE_SOFT){
            if (rs_mode == RS485)
                GPIOC->ODR = GPIO_ODR_ODR_2;//Set rs485/232 line to HIGH
            if (rs_mode == RS232)
                GPIOC->ODR &= ~GPIO_ODR_ODR_2;//Set rs485/232 line to LOW
        }
    }

    rs_t rs_get_rs_mode(void)
    {
        //return rs mode
        //rs232 = 0; rs485 = 1;
        //rs mode line - PC2
        return (rs_t)(RS_MODE);
    }
    
#elif defined (SPI_ENABLED)
    
    void rs_set_fast(void){ }
    void rs_clear_fast(void){   }
    void rs_shutdown(void){ }
    void rs_poweron(void){ }
    rs_t rs_get_rs_mode(void){ return RS_ERROR; }
    void set_rs_mode(rs_t rs_mode){ }
    
#endif

ms_t get_ms_mode(void)
{
    //return master or slave
    return (GPIOB->IDR & GPIO_IDR_IDR_1) ? MASTER : SLAVE;
}

void reset_timers(void)
{
    //rst_gen line - PA2
    GPIOA->ODR |= GPIO_ODR_ODR_2;//Set rs /shdn line to HIGH
    delay_us(50);//50us
    GPIOA->ODR &= !GPIO_ODR_ODR_2;//Set rs /shdn line to LOW
}

uint32_t get_timestamp(void)
{
    return TIM2->CNT;//Get timestamp from timer
}

