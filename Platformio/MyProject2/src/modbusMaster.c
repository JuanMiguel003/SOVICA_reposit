
#include "modbusMaster.h"
#include "etiquetaglo.h"

//#include "modbus.h"

extern UINT16_VAL MBHoldingRegister[256];
extern UINT16_VAL MBInputRegister[256];
extern UINT16_VAL MBCoils;
extern UINT16_VAL MBDiscreteInputs;

// ReadWriteDefinitio Mbpoll1[10];
ReadWriteDefinitio Mbpoll1[NUM_DEVICES];

void TaskModbusMaster1(void *pvParameters)
{
    /*
    Mbpoll1[0].SlaveID = 1;
    Mbpoll1[0].Fuction = 3;
    Mbpoll1[0].Address.Val = 0;
    Mbpoll1[0].Quantity.Val = 10;
    Mbpoll1[0].ScanRate = 1000;
    Mbpoll1[0].TipoDato = Signed;
 */
//   MBHoldingRegister[0].Val = 1;     //id slave=1
//   MBHoldingRegister[1].Val = 3;     //funtion code=03 -- read holding
//   MBHoldingRegister[2].Val = 0;     //direccion read    
//   MBHoldingRegister[3].Val = 10;    //Cantidad de registros(16bits c/u)
//   MBHoldingRegister[4].Val = 1000;  //tiempo de espera scan rate 1000ms
//   MBHoldingRegister[5].Val = 0;     //tipo de dato
//   MBHoldingRegister[6].Val = 0;     //crc


//     Mbpoll1[0].SlaveID = MBHoldingRegister[0].Val;
//     Mbpoll1[0].Fuction = MBHoldingRegister[1].Val;
//     Mbpoll1[0].Address.Val = MBHoldingRegister[2].Val;
//     Mbpoll1[0].Quantity.Val = MBHoldingRegister[3].Val;
//     Mbpoll1[0].ScanRate = MBHoldingRegister[4].Val;
//     Mbpoll1[0].TipoDato = MBHoldingRegister[5].Val;

    Mbpoll1[0].SlaveID =    1;
    Mbpoll1[0].Fuction =    3;
    Mbpoll1[0].Address.Val =    0x00;
    Mbpoll1[0].Quantity.Val =   1;
    Mbpoll1[0].ScanRate =   1000;
    Mbpoll1[0].TipoDato = 0;

    Mbpoll1[0].SlaveID =    2;
    Mbpoll1[0].Fuction =    3;
    Mbpoll1[0].Address.Val =    0x00;
    Mbpoll1[0].Quantity.Val =   1;
    Mbpoll1[0].ScanRate =   1000;
    Mbpoll1[0].TipoDato = 0;
    
    Mbpoll1[0].SlaveID =    3;
    Mbpoll1[0].Fuction =    3;
    Mbpoll1[0].Address.Val =    0x00;
    Mbpoll1[0].Quantity.Val =   3;
    Mbpoll1[0].ScanRate =   1000;
    Mbpoll1[0].TipoDato = 0; 


    UINT16_VAL CRC;

    uint8_t *datoTX = (uint8_t *)malloc(1024);

    // int i = 0;

    while (1)
    {
    for (int i=0;NUM_DEVICES;i++)
    {
        if (Mbpoll1[i].SlaveID != 0)
        {
            datoTX[0] = Mbpoll1[i].SlaveID;
            datoTX[1] = Mbpoll1[i].Fuction;

            datoTX[2] = Mbpoll1[i].Address.byte.HB;
            datoTX[3] = Mbpoll1[i].Address.byte.LB;

            datoTX[4] = Mbpoll1[i].Quantity.byte.HB;
            datoTX[5] = Mbpoll1[i].Quantity.byte.LB;

            CRC.Val = CRC16(datoTX, 6);

            datoTX[6] = CRC.byte.LB;
            datoTX[7] = CRC.byte.HB;

            uart_write_bytes(UART_NUM_1, (const char *)datoTX, 8);

            vTaskDelay(Mbpoll1[i].ScanRate / portTICK_PERIOD_MS);
        }
        else
        {

            // se busca otra id
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }                   // fin del for
    }                   //Fin while(1)
    



    // for (;;)
    // {

        
    //     if (Mbpoll1[i].SlaveID != 0)
    //     {
    //         datoTX[0] = Mbpoll1[i].SlaveID;
    //         datoTX[1] = Mbpoll1[i].Fuction;

    //         datoTX[2] = Mbpoll1[i].Address.byte.HB;
    //         datoTX[3] = Mbpoll1[i].Address.byte.LB;

    //         datoTX[4] = Mbpoll1[i].Quantity.byte.HB;
    //         datoTX[5] = Mbpoll1[i].Quantity.byte.LB;

    //         CRC.Val = CRC16(datoTX, 6);

    //         datoTX[6] = CRC.byte.LB;
    //         datoTX[7] = CRC.byte.HB;

    //         uart_write_bytes(UART_NUM_1, (const char *)datoTX, 8);

    //         vTaskDelay(Mbpoll1[i].ScanRate / portTICK_PERIOD_MS);
    //     }
    //     else
    //     {

    //         // codigo de buscar otra ID ???? i++
    //     }

    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
}