#include "uarts.h"
#include "etiquetaglo.h"


#define tamBUFFER 1024
static QueueHandle_t UARTN_queue;
int estado=0;   //estado=1 se recibio algo por el uart
                //estado=0 no dentra en la rutina read uart


//**************************************
//************* Init UARTs *************
//**************************************

// Función Para iniciar el UART
void initUARTN()
{
    uart_config_t configUART = {
        .baud_rate = MB_SPEED_BAUDR,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };  
    uart_param_config(MB_NUM_UART, &configUART);
    #ifdef MB_PIN_RTS   // comunicacion RTS activa en etiquetaglo.h
        uart_set_pin(MB_NUM_UART, MB_PIN_TX, MB_PIN_RX, MB_PIN_RTS, UART_PIN_NO_CHANGE);    
    #else
        uart_set_pin(MB_NUM_UART, MB_PIN_TX, MB_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    #endif
    uart_driver_install(MB_NUM_UART, tamBUFFER * 2, tamBUFFER * 2, 20, &UARTN_queue, 0);
    #ifdef MB_PIN_RTS
        uart_set_mode(MB_NUM_UART, UART_MODE_RS485_HALF_DUPLEX);
    #endif
    xTaskCreatePinnedToCore(TareaEventosUARTN, "Tarea_para_UART", 1024 * 5, NULL, 12, NULL, 1);
}





//**************************************
//*************** TASKs ****************
//**************************************





void TareaEventosUARTN(void *Parametro)
{
    uart_event_t evento;
    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER);
    for (;;)
    {
        if (xQueueReceive(UARTN_queue, (void *)&evento, (TickType_t)portMAX_DELAY))
        {
            bzero(datoRX, tamBUFFER);
            if (evento.type == UART_DATA)
            {
                printf("\nRECIBIO");
                uart_read_bytes(MB_NUM_UART, datoRX, evento.size, portMAX_DELAY);
                modbusSerial(datoRX, evento.size);
                estado=1;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL);
}

