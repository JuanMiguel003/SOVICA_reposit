#include <Arduino.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#define tamBUFFER 1024

#define TX_RS485_UART0 1 // GPIO1
#define RX_RS485_UART0 3 // GPIO3
#define DE_RS485_UART0 5 // GPIO5

#define TX_RS485_INV 4  //(GPIO_NUM_4)
#define RX_RS485_INV 15 //(GPIO_NUM_15)
#define DE_RS485_INV 2  //(GPIO_NUM_2)

#define TX_RS485_ANA 27 //(GPIO_NUM_27)
#define RX_RS485_ANA 14 //(GPIO_NUM_14)
#define DE_RS485_ANA 12 //(GPIO_NUM_12)

void initUART0(void);
void initUART1(void);
void initUART2(void);

void TareaEventosUART0(void *Parametro);
void TareaEventosUART1(void *Parametro);
void TareaEventosUART2(void *Parametro);

extern void modbusSerial(uint8_t *ByteArray, uint16_t Length);
