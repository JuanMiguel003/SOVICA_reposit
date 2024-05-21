
#include "modbus.h"
#include "driver/gpio.h"
#include "etiquetaglo.h"
#include "freertos/FreeRTOS.h"
#include "def23.h"
extern UINT16_VAL MBHoldingRegister[maxHoldingRegister];
extern UINT16_VAL MBInputRegister[maxInputRegister];
extern UINT16_VAL MBCoils;
extern UINT16_VAL MBDiscreteInputs;

void TaskBlink(void *pvParameters)
{   // SECCION AUXILIAR SOLO PARA PRUEBA
// extern int estado;  //estado=1 -- se recibio data uartN
// extern bool TX;     //Tx=1 -- write about modbus protocol in uart

    gpio_set_direction(GPIO_ROJ, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_AMA, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_VER, GPIO_MODE_OUTPUT);
printf("TaskBlink esta coriendo en el nucleo: %d \n\r", xPortGetCoreID());

while (1)
  {
     // Encender el LED rojo (simulando la luz roja del semáforo)
        gpio_set_level(GPIO_ROJ, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Esperar 2 segundos

        // Apagar el LED rojo y encender el amarillo (simulando la luz amarilla)
        gpio_set_level(GPIO_ROJ, 0);
        gpio_set_level(GPIO_AMA, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Esperar 1 segundo

        // Apagar el LED amarillo y encender el verde (simulando la luz verde)
        gpio_set_level(GPIO_AMA, 0);
        gpio_set_level(GPIO_VER, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Esperar 2 segundos

        // Apagar el LED verde (simulando la luz apagada)
        gpio_set_level(GPIO_VER, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Esperar 1 segundo
    }

  //   switch (estado)
  //   {
  //   case 1:    //RECIBIO ALGO POR EL UART
  //     for (size_t i = 0; i < 4; i++)
  //     {
  //       gpio_set_level(GPIO_LED,1);
  //       vTaskDelay(pdMS_TO_TICKS(300));
  //       gpio_set_level(GPIO_LED,0);
  //       vTaskDelay(pdMS_TO_TICKS(200));        
  //     }
  //     if (TX) {estado=2;
  //               TX=false;}
  //     else estado=0;
  //     break;

  //   case 2:    //TRANSMITIO  ALGO POR EL UART
  //     for (size_t i = 0; i < 2; i++)
  //     {
  //       gpio_set_level(GPIO_LED,1);
  //       vTaskDelay(pdMS_TO_TICKS(900));
  //       gpio_set_level(GPIO_LED,0);
  //       vTaskDelay(pdMS_TO_TICKS(800));        
  //     }
  //     estado=0;
  //     break;

  //   default:
  //     gpio_set_level(GPIO_LED,1);
  //     vTaskDelay(pdMS_TO_TICKS(TIME_ON));

  //     gpio_set_level(GPIO_LED,0);
  //     vTaskDelay(pdMS_TO_TICKS(TIME_OFF));
  //     break;
  //   }
}

void TareaEntradaDatos(void *Parametro)
{
    int16_t datoInAux1 = -100;
    int16_t datoInAux2 = -20;
    int16_t datoInAux3 = 100;
    int16_t datoInAux4 = 0;
    uint8_t cont1 = 0;

    float_VAL datoFloat1;
    // float_VAL datoFloat2;

    datoFloat1.Val = -10.5;
    // datoFloat2.Val = 10.5;

    while (1)
    {
        // Registros 16 bit signo DIRECCIONES 0, 1, 2, 3

        MBInputRegister[0].Val = datoInAux1++;
        MBInputRegister[1].Val = datoInAux2++;
        MBInputRegister[2].Val = datoInAux3++;
        MBInputRegister[3].Val = datoInAux4++;

        // Registros FLOAT DIRECCIONES 100, 101 (Little-endian)

        MBInputRegister[100].byte.HB = datoFloat1.byte.Byte0;
        MBInputRegister[100].byte.LB = datoFloat1.byte.Byte1;
        MBInputRegister[101].byte.HB = datoFloat1.byte.Byte2;
        MBInputRegister[101].byte.LB = datoFloat1.byte.Byte3;

        datoFloat1.Val = datoFloat1.Val + 1.1;

        if (cont1++ == 50)
        {
            datoInAux1 = -100;
            datoInAux2 = -20;
            datoInAux3 = 100;
            datoInAux4 = 0;

            datoFloat1.Val = -10.5;
            cont1 = 0;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TareaSetCoils(void *Parametro)
{
    
#define Coil_1 GPIO_NUM_2
#define Coil_2 GPIO_NUM_16
#define Coil_3 GPIO_NUM_17
#define Coil_4 GPIO_NUM_20

    gpio_set_direction(Coil_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_4, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(Coil_1, MBCoils.bits.b5);
        gpio_set_level(Coil_2, MBCoils.bits.b6);
        gpio_set_level(Coil_3, MBCoils.bits.b7);
        gpio_set_level(Coil_4, MBCoils.bits.b8);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}


