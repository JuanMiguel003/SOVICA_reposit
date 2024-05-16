#include <Arduino.h>
#include <string.h>
#include "defTest.h"
#include "crc16.h"
#include <WiFi.h>
#include "def20.h"
#define TIMEOUT                                     50
#define PORT                                        5000
#define BUFFER_SIZE                                 100
//#define DEBUG

enum Estados{
    CONECTAR,
    ENVIAR_TRAMA_INV,
    RECIBIR_TRAMA_INV,
    ENVIAR_TRAMA_GEN,
    RECIBIR_TRAMA_GEN,
    CONTROL,
    ENVIAR_TRAMA_GUARDAR_FLASH,
    RECIBIR_TRAMA_GUARDAR_FLASH,
    ENVIAR_TRAMA_CONTROL,
    RECIBIR_TRAMA_CONTROL,
}estados;

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)
{

  pinMode(LED_B, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // printf("TaskBlink esata coriendoen el nucleo: %d \n\r", xPortGetCoreID());

  for (;;)
  {
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_B, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void TaskModbusMasterTCP(void *pvParameters) // This is a task.
{
  float_VAL Power;
  UINT32_VAL PowerInv;
  float potencia_inversor = 0;
  float potencia_generador = 0;

  uint8_t timeout = 0;
  uint8_t set_point_min_load = 30;
  uint16_t potencia_nominal_gen = 1500;  // en W
  uint16_t potencia_nominal_inv = 3000;  // en W
  float porcentaje =0;


  //IPAddress server(190, 188, 232, 76); //Argentina
  //IPAddress server(172, 16, 0, 18); //Local
  IPAddress server(172, 16, 0, 32); //Local gatewau
  // Initialize the client library
  WiFiClient client;

  uint8_t ByteArray[BUFFER_SIZE]; // buffer de recepcion de los datos

  //Potencia Activa Fase 1
  const uint8_t trama_generador[8] = {0x02, 0x03, 0x0B, 0xEC, 0x00, 0x02, 0x07, 0xE9};

  //Potencia Salida Inversor
  const uint8_t trama_inversor[8] = {0x01, 0x04, 0x00, 0x0B, 0x00, 0x02, 0x00, 0x09};

  //Guardar cambios de potencia en la flash 
  const uint8_t guardar_flash[8] = {0x01, 0x06, 0x00, 0x02, 0x00, 0x01, 0xE9, 0xCA};

  //trama de control
  uint8_t trama_control[8] = {0x01, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};

  for (;;) //
  {

    switch (estados)
    {
    case CONECTAR:

        #ifdef DEBUG
          printf("EN ESTADO CONECTAR \n\r");
        #endif
          vTaskDelay(1000);

          if (WiFi.status() != WL_CONNECTED)
            initWiFi();
          
          if (client.connect(server, PORT))
            estados = ENVIAR_TRAMA_INV;
      
      break;

    case ENVIAR_TRAMA_INV:

        #ifdef DEBUG
          printf("EN ESTADO ENVIAR_TRAMA_INV \n\r");
        #endif
          if (client.connected())
          {
              client.flush(); //Discard any bytes that have been written to the client but not yet read
              client.write(trama_inversor, 8);
              printf("====INVERSOR===== \n\r");
              printf("TX = \n\r");
              for(uint8_t n=0; n<8; n++)printf("%X ", trama_inversor[n]);
              printf("\n\r");

              estados = RECIBIR_TRAMA_INV;
          }
          else
            estados = CONECTAR;
          
      break;


    case RECIBIR_TRAMA_INV:
        #ifdef DEBUG
          printf("EN ESTADO RECIBIR_TRAMA_INV \n\r");
        #endif
          if (client.available()>=9) // Verifica si ha recibido datos por TCP
          {
            uint8_t cantBytes = client.available(); // Se almacena la cant de bytes recibidos

            memset(ByteArray, 0x00, BUFFER_SIZE);

            client.read(ByteArray, cantBytes); // Se guarda en memoria la cantidad recibida
             if(CRC16(ByteArray, cantBytes)==0 && ByteArray[0]==trama_inversor[0])
              {
                PowerInv.byte.MB = ByteArray[3];
                PowerInv.byte.UB = ByteArray[4];
                PowerInv.byte.HB = ByteArray[5];
                PowerInv.byte.LB = ByteArray[6];
                printf("====INVERSOR===== \n\r");
                printf("RX = \n\r");
                for(uint8_t n=0; n<cantBytes; n++)printf("%X ", ByteArray[n]);
                printf("\n\r");
                printf("Potencia Activa Inversor (w) = %f\n\r",(float) PowerInv.Val*0.1 );
                potencia_inversor = PowerInv.Val*0.1;

                client.flush(); //Discard any bytes that have been written to the client but not yet read
                
                //estados = ENVIAR_TRAMA_GEN;
                estados = ENVIAR_TRAMA_GEN;
                vTaskDelay(5000);
              }
              else
                estados = ENVIAR_TRAMA_INV;  // Si falla el CRC 
          
          }
          vTaskDelay(100);
          if (timeout == TIMEOUT)
          {
            estados = ENVIAR_TRAMA_INV;
            timeout=0;
          }
          else
            timeout++;
      break;

    case ENVIAR_TRAMA_GEN:
        #ifdef DEBUG
          printf("EN ESTADO ENVIAR_TRAMA_GEN \n\r");
        #endif
          if (client.connected())
          {
              client.flush(); //Discard any bytes that have been written to the client but not yet read
              client.write(trama_generador, 8);
              printf("====GENERADOR===== \n\r");
              printf("TX = \n\r");
              for(uint8_t n=0; n<8; n++)printf("%X ", trama_generador[n]);
              printf("\n\r");

              estados = RECIBIR_TRAMA_GEN;
          }
          else
            estados = CONECTAR;
          
    break;

    case RECIBIR_TRAMA_GEN:
        #ifdef DEBUG
            printf("EN ESTADO RECIBIR_TRAMA_GEN \n\r");
        #endif

          if (client.available()>=9) // Verifica si ha recibido datos por TCP
          {
            uint8_t cantBytes = client.available(); // Se almacena la cant de bytes recibidos

            memset(ByteArray, 0x00, BUFFER_SIZE);

            client.read(ByteArray, cantBytes); // Se guarda en memoria la cantidad recibida
              if(CRC16(ByteArray, cantBytes)==0 && ByteArray[0]==trama_generador[0])
              {
                Power.byte.Byte1 = ByteArray[3];
                Power.byte.Byte0 = ByteArray[4];
                Power.byte.Byte3 = ByteArray[5];
                Power.byte.Byte2 = ByteArray[6];
                printf("====GENERADOR===== \n\r");
                printf("RX = \n\r");
                for(uint8_t n=0; n<cantBytes; n++)printf("%X ", ByteArray[n]);
                printf("\n\r");

                printf("Potencia Activa Generador(w) = %f\n\r", Power.Val*1000);
                potencia_generador = Power.Val*1000;

                client.flush(); //Discard any bytes that have been written to the client but not yet read
                
                estados = ENVIAR_TRAMA_INV;
                vTaskDelay(5000);
                //estados = CONTROL;
              }
              else
                estados = ENVIAR_TRAMA_GEN; // Si falla el CRC 
          }
          vTaskDelay(100);
          if (timeout == TIMEOUT)
          {
            estados = ENVIAR_TRAMA_GEN;
            timeout=0;
          }
          else
            timeout++;
      break;

      case CONTROL:

          {
            #ifdef DEBUG
                printf("EN ESTADO CONTROL \n\r");
            #endif
              
              float potencia_total = potencia_inversor + potencia_generador;
              float potencia_max_inversor = potencia_total - (potencia_nominal_gen*set_point_min_load)/100;
              porcentaje = (100 * potencia_max_inversor)/potencia_nominal_inv;

              if (porcentaje < 0) //Aplicar filtro
                  porcentaje =0;
              else if(porcentaje > 100)
                  porcentaje = 100;
              else
                  porcentaje = (uint8_t)porcentaje;
              printf("porcentaje a set = %d \n\r", (uint8_t)porcentaje);
              
              estados = ENVIAR_TRAMA_GUARDAR_FLASH;
          }

      break;

      case ENVIAR_TRAMA_GUARDAR_FLASH:
      
          #ifdef DEBUG
            printf("EN ESTADO ENVIAR_TRAMA_GUARDAR_FLASH \n\r");
          #endif
          if (client.connected())
          {
              client.write(guardar_flash, 8);

              estados = RECIBIR_TRAMA_GUARDAR_FLASH;
          }
          else
            estados = CONECTAR;
          
      break;

      case RECIBIR_TRAMA_GUARDAR_FLASH:

          if (client.available()>=8) // Verifica si ha recibido datos por TCP
          {
            uint8_t cantBytes = client.available(); // Se almacena la cant de bytes recibidos
                                                    // Serial.print("Cant de bytes: ");
            memset(ByteArray, 0x00, BUFFER_SIZE);

            client.read(ByteArray, cantBytes); // Se guarda en memoria la cantidad recibida
            if( !memcmp(guardar_flash, ByteArray, 8)) // compara los 2 array
              estados = ENVIAR_TRAMA_CONTROL;
            else  
              estados = ENVIAR_TRAMA_GUARDAR_FLASH;
          }
          vTaskDelay(100);
          if (timeout == TIMEOUT)
          {
            estados = ENVIAR_TRAMA_GUARDAR_FLASH;
            timeout=0;
          }
          else
            timeout++;

      break;

      case ENVIAR_TRAMA_CONTROL:

          #ifdef DEBUG
            printf("EN ESTADO ENVIAR_TRAMA_CONTROL \n\r");
          #endif
          if (client.connected())
          {
              trama_control[5] = (uint8_t)porcentaje;
              uint16_t crc = CRC16(trama_control, 6);
              trama_control[6] = (crc & 0xff); //Byte menos significativo
              trama_control[7] = ((crc >> 8) & 0xff); //Byte mas significativo

              client.write(trama_control, 8);

              estados = RECIBIR_TRAMA_CONTROL;
          }
          else
            estados = CONECTAR;

      break;

      case RECIBIR_TRAMA_CONTROL:

          #ifdef DEBUG
            printf("EN ESTADO RECIBIR_TRAMA_CONTROL \n\r");
          #endif
          if (client.available()>=8) // Verifica si ha recibido datos por TCP
          {
            uint8_t cantBytes = client.available(); // Se almacena la cant de bytes recibidos

            memset(ByteArray, 0x00, BUFFER_SIZE);

            client.read(ByteArray, cantBytes); // Se guarda en memoria la cantidad recibida
            if( !memcmp(trama_control,(const uint8_t*) ByteArray, 8)) // compara los 2 array
              {
              estados = ENVIAR_TRAMA_INV; //
              vTaskDelay(5000);
              }
            else  
              estados = ENVIAR_TRAMA_CONTROL;
          }
          vTaskDelay(100);
          if (timeout == TIMEOUT)
          {
            estados = ENVIAR_TRAMA_CONTROL;
            timeout=0;
          }
          else
            timeout++;
      
      break;

      default:

          #ifdef DEBUG
            printf("EN ESTADO DESCONOCIDO \n\r");
          #endif
            estados = CONECTAR;

      break;
    } // maquina de estado
  } // for
} //task

void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin("HOLETB 2.4_EXT", "primitiva");
  //Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    // Serial.print('.');
    vTaskDelay(1000);
  }
  // Serial.println(WiFi.localIP());
}