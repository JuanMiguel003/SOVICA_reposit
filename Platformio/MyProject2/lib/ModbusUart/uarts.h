
#include "modbus.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
// #include <etiquetaglo.h>

void initUARTN();
void TareaEventosUARTN(void *Parametro);