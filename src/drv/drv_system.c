#include "board.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_conf.h"

typedef struct gpio_config_t
{
    GPIO_TypeDef *gpio;
    uint16_t pin;
    GPIOMode_TypeDef mode;
} gpio_config_t;

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;

    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    }
    while (ms != sysTickUptime);

    return (ms * 1000) + ((SystemCoreClock / 1000) - cycle_cnt) / (SystemCoreClock / 1000000);
    //return (ms * 1000) + (72000 - cycle_cnt) / 72; //TODO: fix this for 80mhz (sysclk?) i think this works now
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(void)
{
	ErrorStatus HSEStartUpStatus;
    GPIO_InitTypeDef GPIO_InitStructure;
    gpio_config_t gpio_cfg[] =
    {
        { LEDR_GPIO, LEDR_PIN, GPIO_Mode_Out_PP },
        { LEDG_GPIO, LEDG_PIN, GPIO_Mode_Out_PP },
        // { LEDB_GPIO, LEDB_PIN, GPIO_Mode_Out_PP },
    };
    uint8_t gpio_count = sizeof(gpio_cfg) / sizeof(gpio_cfg[0]);
    uint32_t i;

    // This is needed because some shit inside Keil startup fucks with SystemCoreClock, setting it back to 72MHz even on HSI.
    // TODO this
    //SystemCoreClockUpdate();
    //SetSysClock();
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* ADCCLK = PCLK2/8 */
        RCC_ADCCLKConfig(RCC_PCLK2_Div8);

        /* PLLCLK = 12MHz * 6 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    {
        /* If HSE fails to start-up, the application will have wrong clock configuration.
           User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }



    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // // Turn off JTAG port 'cause we're using the GPIO for leds
    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    // Configure gpio
    for (i = 0; i < gpio_count; i++)
    {
        GPIO_InitStructure.GPIO_Pin = gpio_cfg[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode = gpio_cfg[i].mode;
        GPIO_Init(gpio_cfg[i].gpio, &GPIO_InitStructure);
    }

    LEDR_OFF();
    LEDG_OFF();
    // LEDB_OFF();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // sleep for 100ms
    delay(100);

    /* USB_DISCONNECT used as USB pull-up */
    GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
    USB_DISCONNECT->BSRR = USB_DISCONNECT_PIN;

    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();

    while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader)
    {
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

// Added USB Stuff

uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE];
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->Current_Configuration != 0)
    {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
    else
    {
        bDeviceState = ATTACHED;
    }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        USB_DevConnect();
    }
    else
    {
        USB_DevDisconnect();
    }
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{

    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State != 1)
    {
        if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
        {
            USART_Rx_ptr_out = 0;
        }

        if (USART_Rx_ptr_out == USART_Rx_ptr_in)
        {
            USB_Tx_State = 0;
            return;
        }

        if (USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
        {
            USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
        }
        else
        {
            USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
        }

        if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

            USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
            USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
        }
        else
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = USART_Rx_length;

            USART_Rx_ptr_out += USART_Rx_length;
            USART_Rx_length = 0;
        }
        USB_Tx_State = 1;

#ifdef USE_STM3210C_EVAL
        USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);
#else
        UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
        SetEPTxCount(ENDP1, USB_Tx_length);
        SetEPTxValid(ENDP1);
#endif
    }

}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USB_Send_Data(char c)
{
    USART_Rx_Buffer[USART_Rx_ptr_in] = c;
    USART_Rx_ptr_in++;

    /* To avoid buffer overflow */
    if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_in = 0;
    }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(__IO uint32_t *)(0x1FFFF7E8);
    Device_Serial1 = *(__IO uint32_t *)(0x1FFFF7EC);
    Device_Serial2 = *(__IO uint32_t *)(0x1FFFF7F0);

    Device_Serial0 += Device_Serial2;

    if (Device_Serial0 != 0)
    {
        IntToUnicode (Device_Serial0, (uint8_t*)&Virtual_Com_Port_StringSerial[2] , 8);
        IntToUnicode (Device_Serial1, (uint8_t*)&Virtual_Com_Port_StringSerial[18], 4);
    }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
    uint8_t idx = 0;

    for ( idx = 0 ; idx < len ; idx ++)
    {
        if ( ((value >> 28)) < 0xA )
        {
            pbuf[ 2 * idx] = (value >> 28) + '0';
        }
        else
        {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }

        value = value << 4;

        pbuf[ 2 * idx + 1] = 0;
    }
}
