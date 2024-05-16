#include <Arduino.h>

#include "defTest.h"
#include "uarts.h"
#include "modbusMaster.h"

void setup()
{

  initUART0();
  initUART1();
  initUART2();

  initControl();

  xTaskCreatePinnedToCore(TaskBlink, "TaskBlink", 4096, NULL, 2, NULL, 0);

  xTaskCreatePinnedToCore(TaskModbusMaster1, "TaskMaster1", 4096, NULL, 2, NULL, 1);

  xTaskCreatePinnedToCore(TaskModbusMasterTCP, "ModbusMaster", 4096*4, NULL, 3, NULL, 1);
}

void loop()
{
}

