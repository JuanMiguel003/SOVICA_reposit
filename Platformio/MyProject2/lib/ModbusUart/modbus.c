#include "modbus.h"
#include "crc16.h"
#include "etiquetaglo.h"
#include <stdbool.h>
#include "def23.h"


bool TX=false;


// uint8_t ByteArray[260]; // buffer de recepcion de los datos recibidos de los clientes
UINT16_VAL MBHoldingRegister[maxHoldingRegister];
UINT16_VAL MBInputRegister[maxInputRegister];
UINT16_VAL MBCoils;
UINT16_VAL MBDiscreteInputs;

//**************************************
//************* Funciones **************
//**************************************

void modbusSerial(uint8_t *ByteArray, uint16_t Length)
{
    uint8_t byteFN = ByteArray[MB_SER_FUNC]; // maquina de estado
    UINT16_VAL Start;

    UINT16_VAL WordDataLength;

    UINT16_VAL CRC;
    uint8_t Id_slave;
    uint8_t Bytes_Data;

    if (CRC16(ByteArray, Length) == 0)
    {

        switch (byteFN)
        {
        case MB_FC_NONE:

            break;

        case MB_FC_READ_COILS: // 01 Read Coils

            // La direccion = 0 y el numero = 16  de los Coils son fijo

            // numero de bytes de datos de respuesta
            ByteArray[2] = 2;

            ByteArray[3] = MBCoils.byte.LB;
            ByteArray[4] = MBCoils.byte.HB;

            // CRC
            CRC.Val = CRC16(ByteArray, 5);

            ByteArray[5] = (CRC.byte.LB);
            ByteArray[6] = (CRC.byte.HB);

            uart_write_bytes(MB_NUM_UART, (const char *)ByteArray, 7);

            break;

        case MB_FC_READ_DISCRETE_INPUTS: // funcion 02: Read Discrete Inputs

            break;

        case MB_FC_READ_REGISTERS: // 03 Read Holding Registers

            Id_slave=ByteArray[0];
            Bytes_Data=ByteArray[2];


            printf("\nId del slave: %u\t N-Bytes: %u\t DATA: ",Id_slave,Bytes_Data);
            switch (Id_slave)
            {
            case 1:
            for (uint16_t i = 0; i <(Bytes_Data/2); i++) // datos de respuesta
            {
                MBHoldingRegister[20+i].byte.HB= ByteArray[3 + i * 2];                
                MBHoldingRegister[20+i].byte.LB= ByteArray[4 + i * 2];
                printf(" -%d",MBHoldingRegister[20+i].Val);
            }            
                break;
            case 2:
            for (uint16_t i = 0; i <(Bytes_Data/2); i++) // datos de respuesta
            {
                MBHoldingRegister[40+i].byte.HB= ByteArray[3 + i * 2];                
                MBHoldingRegister[40+i].byte.LB= ByteArray[4 + i * 2];
                printf(" -%d",MBHoldingRegister[40+i].Val);
            }    
                break;
            case 3:
            for (uint16_t i = 0; i <(Bytes_Data/2); i++) // datos de respuesta
            {
                MBHoldingRegister[60+i].byte.HB= ByteArray[3 + i * 2];                
                MBHoldingRegister[60+i].byte.LB= ByteArray[4 + i * 2];
                printf(" -%d",MBHoldingRegister[60+i].Val);
            }    
                break;            
            default:
                break;
            }
            TX=true;

            // // direccion de cominzo Modbus
            // Start.byte.HB = ByteArray[2];
            // Start.byte.LB = ByteArray[3];

            // // numero de datos
            // WordDataLength.byte.HB = ByteArray[4];
            // WordDataLength.byte.LB = ByteArray[5];

            // // numero de bytes de datos de respuesta
            // ByteArray[2] = WordDataLength.Val * 2;

            // for (uint16_t i = 0; i < WordDataLength.Val; i++) // datos de respuesta
            // {
            //     ByteArray[3 + i * 2] = MBHoldingRegister[i + Start.Val].byte.HB;
            //     ByteArray[4 + i * 2] = MBHoldingRegister[i + Start.Val].byte.LB;
            // }

            // // CRC
            // CRC.Val = CRC16(ByteArray, ByteArray[2] + 3);
            // ByteArray[ByteArray[2] + 3] = (CRC.byte.LB);
            // ByteArray[ByteArray[2] + 4] = (CRC.byte.HB);

            // // rs485(ByteArray, ByteArray[2] + 5);

            // uart_write_bytes(MB_NUM_UART, (const char *)ByteArray, ByteArray[2] + 5);

            // TX=true;

            break;

        case MB_FC_READ_INPUT_REGISTERS: // 04 Read Input Registers Registers
                                         // direccion de cominzo Modbus
            Start.byte.HB = ByteArray[2];
            Start.byte.LB = ByteArray[3];

            // numero de datos
            WordDataLength.byte.HB = ByteArray[4];
            WordDataLength.byte.LB = ByteArray[5];

            // numero de bytes de datos de respuesta
            ByteArray[2] = WordDataLength.Val * 2;

            for (uint16_t i = 0; i < WordDataLength.Val; i++) // datos de respuesta
            {
                ByteArray[3 + i * 2] = MBInputRegister[i + Start.Val].byte.HB;
                ByteArray[4 + i * 2] = MBInputRegister[i + Start.Val].byte.LB;
            }

            // CRC
            CRC.Val = CRC16(ByteArray, ByteArray[2] + 3);
            ByteArray[ByteArray[2] + 3] = (CRC.byte.LB);
            ByteArray[ByteArray[2] + 4] = (CRC.byte.HB);

            uart_write_bytes(MB_NUM_UART, (const char *)ByteArray, ByteArray[2] + 5);

            break;

        case MB_FC_WRITE_COIL: // 05 Write COIL

            // Respuesta es igual que la pregunta
            uart_write_bytes(MB_NUM_UART, (const char *)ByteArray, 8);

            // direccion del punto binario
            Start.byte.HB = ByteArray[2];
            Start.byte.LB = ByteArray[3];

            // valor binario del bit
            // 0xFF 0x00 --> On        0x00 0x00  --> Off
            WordDataLength.byte.HB = ByteArray[4];
            WordDataLength.byte.LB = ByteArray[5];

            bitWrite(MBCoils.Val, Start.Val, (WordDataLength.Val == 0xFF00 ? 1 : 0));

            break;

        case MB_FC_WRITE_REGISTER: // 06 Write Holding Register

            // direccion de cominzo Modbus
            Start.byte.HB = ByteArray[2];
            Start.byte.LB = ByteArray[3];

            MBHoldingRegister[Start.Val].byte.HB = ByteArray[4];
            MBHoldingRegister[Start.Val].byte.LB = ByteArray[5];


            uart_write_bytes(MB_NUM_UART, (const char *)ByteArray, 8);

            break;

        case MB_FC_WRITE_MULTIPLE_COILS: // 15 Write Coils

            break;

        case MB_FC_WRITE_MULTIPLE_REGISTERS: // 16 Write Holding Registers

            break;
        }
    }
    else printf("\nError CRC");
}
