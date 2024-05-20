#include <stdint.h>
#include "crc16.h"
#include "def23.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define Signed 0
#define Unsigned 1
#define Hex 2
#define Binary 3
#define Float 4
#define Double 5

#define maxMInputRegister 256
#define maxMHoldingRegister 256


typedef union
{
    uint16_t Val;
    
    struct
    {
        uint8_t LB;
        uint8_t HB;
    } byte;

} uint16_VAL;

typedef struct
{
    uint8_t SlaveID;
    uint8_t Fuction;
    uint16_VAL Address;
    uint16_VAL Quantity;
    uint16_t ScanRate;
    uint8_t TipoDato;
    
}ReadWriteDefinitio;



void TaskModbusMaster1(void *pvParameters);
