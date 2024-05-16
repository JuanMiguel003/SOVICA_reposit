/*     #################################  modbus master  #########################################*/
// nombre: Juan Contreras

#include "uarts.h"
#include "tareas.h"
#include "driver/gpio.h"
#include "etiquetaglo.h"

//SECCION DE DEFINES 
#define GPIO_LED GPIO_NUM_23


void app_main()
{

  initUARTN();

  xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 2, NULL, 3, NULL, 1);

  xTaskCreatePinnedToCore(TareaSetCoils, "Tarea_Salida_Binarias", 1024 * 2, NULL, 2, NULL, 1);






// SECCION AUXILIAR SOLO PARA PRUEBA DE COMPILE
gpio_set_direction(GPIO_LED,GPIO_MODE_OUTPUT);
extern int estado;
extern bool TX;
while (1)
  {
    switch (estado)
    {
    case 1:    //RECIBIO ALGO POR EL UART
      for (size_t i = 0; i < 4; i++)
      {
        gpio_set_level(GPIO_LED,1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(GPIO_LED,0);
        vTaskDelay(pdMS_TO_TICKS(200));        
      }
      if (TX) {estado=2;
                TX=false;}
      else estado=0;
      break;
    case 2:    //TRANSMITIO  ALGO POR EL UART
      for (size_t i = 0; i < 2; i++)
      {
        gpio_set_level(GPIO_LED,1);
        vTaskDelay(pdMS_TO_TICKS(900));
        gpio_set_level(GPIO_LED,0);
        vTaskDelay(pdMS_TO_TICKS(800));        
      }
      estado=0;
      break;
    default:
      gpio_set_level(GPIO_LED,1);
      vTaskDelay(pdMS_TO_TICKS(TIME_ON));

      gpio_set_level(GPIO_LED,0);
      vTaskDelay(pdMS_TO_TICKS(TIME_OFF));
      break;
    }
  }       //FIN DEL WHILE(LED)
}