#include "uarts.h"
#include "tareas.h"
#include "modbusMaster.h"






void app_main()
{

  initUARTN();



  xTaskCreatePinnedToCore(TaskBlink, "TaskBlink", 4096, NULL, 2, NULL, 0);

  xTaskCreatePinnedToCore(TaskModbusMaster1, "TaskMaster1", 4096, NULL, 5, NULL, 1);



  xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 2, NULL, 2, NULL, 1);

  xTaskCreatePinnedToCore(TareaSetCoils, "Tarea_Salida_Binarias", 1024 * 2, NULL, 3, NULL, 1);


  
  

}