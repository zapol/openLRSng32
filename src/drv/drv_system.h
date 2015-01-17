#pragma once

#define USART_RX_DATA_SIZE   1024

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// failure
void failureMode(uint8_t mode);

// bootloader/IAP
void systemReset(bool toBootloader);

// USB Stuff
#define USB_DISCONNECT                      GPIOB
#define USB_DISCONNECT_PIN                  GPIO_Pin_5
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOB
#define USB_DevConnect()                    (USB_DISCONNECT->CRH &= ~(0x01<<20))
#define USB_DevDisconnect()                 (USB_DISCONNECT->CRH |= 0x01<<20)

void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USB_Send_Data(char);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void IntToUnicode (uint32_t, uint8_t*, uint8_t);
