#define LED_B 2
#define LED1 33
#define LED2 25
#define LED3 26

void TaskBlink(void *pvParameters);
void TaskModbusMasterTCP(void *pvParameters);


void initWiFi();
extern void initControl(void);