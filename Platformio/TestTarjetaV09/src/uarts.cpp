#include "uarts.h"
#include "crc16.h"
//#include "modbus.h"

static QueueHandle_t uart0_queue;

static QueueHandle_t uart2_queue;
static QueueHandle_t uart1_queue;

//**************************************
//************* Init UARTs *************
//**************************************

// Función Para iniciar el UART0
void initUART0()
{
    uart_config_t configUART0 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &configUART0);
    uart_set_pin(UART_NUM_0, TX_RS485_UART0, RX_RS485_UART0, (-1) /*DE_RS485_UART0*/, (-1));
    uart_driver_install(UART_NUM_0, tamBUFFER * 2, 0, 20, &uart0_queue, 0);
    //uart_set_mode(UART_NUM_0, UART_MODE_RS485_HALF_DUPLEX);

    pinMode(DE_RS485_UART0, OUTPUT);
    digitalWrite(DE_RS485_UART0, LOW); // RS485 modo Rx

    xTaskCreatePinnedToCore(TareaEventosUART0, "Tarea_para_UART0", 1024 * 5, NULL, 12, NULL, 1);
}

// Función Para iniciar el UART1
void initUART1(void)
{
    const uart_config_t uart1_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, TX_RS485_INV, RX_RS485_INV, DE_RS485_INV, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, tamBUFFER * 2, tamBUFFER * 2, 20, &uart1_queue, 0);

    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);

    xTaskCreatePinnedToCore(TareaEventosUART1, "Tarea_para_UART1", 1024 * 5, NULL, 12, NULL, 1);
}

// Función Para iniciar el UART2
void initUART2(void)
{
    const uart_config_t uart2_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_2, &uart2_config);
    uart_set_pin(UART_NUM_2, TX_RS485_ANA, RX_RS485_ANA, DE_RS485_ANA, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, tamBUFFER * 2, tamBUFFER * 2, 20, &uart2_queue, 0);


    uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);

    xTaskCreatePinnedToCore(TareaEventosUART2, "Tarea_para_UART2", 1024 * 5, NULL, 12, NULL, 1);

}

//**************************************
//*************** TASKs ****************
//**************************************

void TareaEventosUART0(void *Parametro)
{
    uart_event_t evento;
    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER);

    for (;;)
    {
        if (xQueueReceive(uart0_queue, (void *)&evento, (portTickType)portMAX_DELAY))
        {
            bzero(datoRX, tamBUFFER);
            if (evento.type == UART_DATA)
            {
                uart_read_bytes(UART_NUM_0, datoRX, evento.size, portMAX_DELAY);
                modbusSerial(datoRX, evento.size);

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL);
}

void TareaEventosUART1(void *Parametro)
{
    uart_event_t evento;
    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER);

    for (;;)
    {
        if (xQueueReceive(uart1_queue, (void *)&evento, (portTickType)portMAX_DELAY))
        {
            bzero(datoRX, tamBUFFER);
            if (evento.type == UART_DATA)
            {
                uart_read_bytes(UART_NUM_1, datoRX, evento.size, portMAX_DELAY);
                
                uart_write_bytes(UART_NUM_0, (const char *)datoRX, evento.size);

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL);
}

void TareaEventosUART2(void *Parametro)
{
    uart_event_t evento;
    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER);

    for (;;)
    {
        if (xQueueReceive(uart2_queue, (void *)&evento, (portTickType)portMAX_DELAY))
        {
            bzero(datoRX, tamBUFFER);
            if (evento.type == UART_DATA)
            {
                uart_read_bytes(UART_NUM_2, datoRX, evento.size, portMAX_DELAY);

                uart_write_bytes(UART_NUM_2, (const char *)datoRX, evento.size);

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL);
}
//**************************************
//************* Funciones **************
//**************************************
