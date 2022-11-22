//Nome ALUNO A- João Francisco Carreira Bento
//Numero ALUNO A- 2181137
//Nome ALUNO B- Leandro Pereira Filipe
//Numero ALUNO B- 2181116
//IPLEIRIA - Instituto Politécnico de Leiria
//ESTG - Escola Superior de Tecnologia e Gestão
//LEAU- Licenciatura em Engenharia Automóvel
//SEEV - Sistemas Elétricos e Eletrónicos de Veículos
 
/*TP1: Pretende-se  neste  trabalho  prático  a  implementação  de um  Sistema de Medição de um sistema de refrigeração
* , utilizando um sistema operativo de tempo real FreeRTOS.*/

/*Inclusão das Bibliotecas utilizadas*/
#include "Arduino.h"
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

/*Definição de zonas de debugging*/
#define DEBUG_PRINT_ALL
#define DEBUG_PRINT_1

//Definição de constantes associadas aos pinos de ligação de cada um dos componentes utilizados
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DHTPIN 33        // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT11 // Sensor DHT 11
#define LED_PIN_Red 4		 //PIN 4 - Led VERMELHO

//Definição do pino e das constantes associadas ao conversor digital-analógico
#define THERMISOR_PIN 32 //PIN 32 - Sensor Analógico de Temperatura
#define ADC_1_4 32
#define ADC_RESOLUTION 10
#define VREF_PLUS  3.3
#define VREF_MINUS  0.0

#define MAX_TEMP 75
#define MED_TEMP 60

//INTERRUPT PIN
const uint8_t interruptPin = 27; //PIN 27 para butão de interrupção

/*-TASK FUNCTIONS-*/
void vTaskBrain(void *pvParameters);      //Função Tarefa Brain
void vTask_Temp(void *pvParameters); //Função Tarefa que lê temperatura e mostra no LCD
void vTask_LCD(void *pvParameters);
void vTaskFAN(void *pvParameters);	  //Função Tarefa aciona o Buzzer
void vTaskLED(void *pvParameters);	      //Função Tarefa que controla os LED
void vTaskADC(void *pvParameters);	      //Função Tarefa Sensor Temp. Analógico

TaskHandle_t xHandleLED;
TaskHandle_t xHandleADC;

/*-INTERRUPTS-*/
static void vHandlerTask(void *pvParameters);
void IRAM_ATTR vInterruptHandler();

/*-SEMAPHORES-*/
SemaphoreHandle_t xBinarySemaphore;

/*-QUEUES-*/
QueueHandle_t xQueueTemp, xQueueTempBRAIN, xQueueInterruptBRAIN, xQueueLCD, xQueueADC;

/*-MUTEX-*/
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t xMutex;

void setup() {
	Serial.begin(115200);
	//Criar QUEUE's
	xQueueTempBRAIN = xQueueCreate(1, sizeof(float));
	xQueueTemp = xQueueCreate(1, sizeof(float));
	xQueueLCD = xQueueCreate(1, sizeof(float));
	xQueueADC = xQueueCreate(1, sizeof(float));
	xQueueInterruptBRAIN = xQueueCreate(1, sizeof(float));
	//Criar Semáforo Binário
	vSemaphoreCreateBinary(xBinarySemaphore);
	xMutex = xSemaphoreCreateMutex();
    //Criação tarefas
	xTaskCreatePinnedToCore(vTaskBRAIN, "Brain", 1024, NULL, 4, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Temp, "Temp", 1024, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(vTask_LCD, "LCD", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTaskLED, "LED", 1024, NULL, 1, &xHandleLED, 1);
	xTaskCreatePinnedToCore(vTaskADC, "ADC", 1024, NULL, 1, &xHandleADC, 1);
	pinMode(interruptPin, INPUT_PULLUP); //Definir PIN da interrupção com pullup interna
	attachInterrupt(digitalPinToInterrupt(interruptPin), vInterruptHandler,
			FALLING);

	if (xBinarySemaphore != NULL) {
		xTaskCreatePinnedToCore(vHandlerTask, "Handler", 1000, NULL, 5, NULL,
				0); //Criar HandlerTask no Core
	}
}

void vTaskBRAIN(void *pvParameters) { //Inicio da Tarefa Brain
	TickType_t xLastWakeTime;             //Variavel para determinar nº de ticks
	xLastWakeTime = xTaskGetTickCount();
	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	float temperature, TempBrain, tempANALOG;
	int Emergencia;
	int ledChannel_Red = 5;
	int resolution = 8;
	int dutyCycle_Red = 100;

	for (;;) {
		xQueueReceive(xQueueTemp, &temperature, 0); //Receber Temperatura do DHT11
		xQueueReceive(xQueueInterruptBRAIN, &Emergencia, 0); //Receber estado do botão de emergencia

#ifdef DEBUG_PRINT_ALL
		Serial.print("Emergencia:"); //Verificar estado do botão em porta série
		Serial.println(Emergencia);
#endif

		TempBrain = (temperature * 7.66666666) - 100; //Converter Temperatura Ambiente em Temp. aproximada do Motor
#ifdef DEBUG_PRINT_ALL
		Serial.print(F("Temperature: "));
		Serial.println(TempBrain);
#endif

		if (TempBrain >= MED_TEMP) {
			xTaskCreatePinnedToCore(vTaskFAN, "Fan", 1024, NULL, 1, NULL, 0); //Criar tarefa Buzzer no Core 0 apenas
		}
#ifdef DEBUG_PRINT_ALL
		Serial.print("Emergencia:"); //Verificar estado do botão em porta série
		Serial.println(Emergencia);
#endif

		xQueueOverwrite(xQueueTempBRAIN, &TempBrain); //Enviar Temp. aproximada do Motor
#ifdef DEBUG_PRINT_ALL
		Serial.println(F("BRAIN's: OK"));
#endif
		vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS)); //Tarefa ocorre durante 250ms
	}
}                                           //Fim da Tarefa Brain

static void vHandlerTask(void *pvParameters) {  //Inicio Handler Task
	int Emergencia = 0;
	float TempBrain;

	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	for (;;) {
		xQueuePeek(xQueueTempBRAIN, &TempBrain, 0); //Receber valor de TempBRAIN

		if (TempBrain >= MAX_TEMP) { //Alterar estado do botão
			Emergencia = 1;
#ifdef DEBUG_PRINT_1
			Serial.print("Emergencia:"); //Verificar estado do botão em porta série
			Serial.println(Emergencia);
#endif
		} else
			Emergencia = 0;

#ifdef DEBUG_PRINT_ALL
		Serial.println(F("HandlerTask's: OK"));
#endif
		xQueueSendToBack(xQueueInterruptBRAIN, &Emergencia, 0); //Enviar estado do botão para a brain
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}                                          //Fim da Handler Task

void vInterruptHandler() {                                //Inicio Interrupção
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	int Emergencia = 0;

	vTaskSuspendAll(); //Suspende o sistema enquanto o botão associado à interrupção está a ser premido

#ifdef DEBUG_PRINT_1
	Serial.println("Interrupcao gerada");
#endif
}                                         //Fim Interrupção

void vTask_LCD(void *pvParameters) {
	TickType_t xLastWakeTime;             //Variavel para determinar nº de ticks
	xLastWakeTime = xTaskGetTickCount();
	Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
	float TempBrain, tempANALOG;

	xSemaphoreTake(xMutex, 0);  //Mutex para proteger comunicação I2C

	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { //Detetar se está a ocorrer comunicação
#ifdef DEBUG_PRINT_ALL
		Serial.println(F("SSD1306 allocation failed"));
#endif
	}
	display.clearDisplay();                      //Limpar Display
	display.setTextColor(WHITE, BLACK);				//Selecionar cor do texto
	display.setRotation(0);
	display.setTextSize(2);				//Selecionar tamanho do texto
	display.setCursor(0, 0);	//Selecionar onde imprime o texto no display
	display.print("Mot. Temp.");

	display.setTextSize(2);				//Selecionar tamanho do texto
	display.setCursor(0, 35);	//Selecionar onde imprime o texto no display
	display.print("ADC Temp.");

	xSemaphoreGive(xMutex);

	for (;;) {

		xQueuePeek(xQueueTempBRAIN, &TempBrain, 0); //Receber valor de TempBRAIN
		xSemaphoreTake(xMutex, portMAX_DELAY);

		display.setTextSize(1.75);				//Selecionar tamanho do texto
		display.setCursor(0, 18);	//Selecionar onde imprime o texto no display
		display.print(TempBrain);	//Mostrar Temperatura no display
		display.print(" ");
		display.setTextSize(1);
		display.cp437(true);                       //Desenhar "º" no display
		display.write(167);
		display.setTextSize(1.75);
		display.print("C");
		if (TempBrain > MAX_TEMP) {
			display.setTextSize(1.75);             //Selecionar tamanho do texto
			display.setCursor(70, 18); //Selecionar onde imprime o texto no display
			display.print("PERIGO");
		}

#ifdef DEBUG_PRINT_ALL
		Serial.println(F("LCD's: OK"));
#endif
#ifdef DEBUG_PRINT_ALL
		Serial.println(TempBrain);
#endif

		xQueueReceive(xQueueADC, &tempANALOG, 0);

		display.setTextSize(1.75);				//Selecionar tamanho do texto
		display.setCursor(0, 50);	//Selecionar onde imprime o texto no display
		display.print(tempANALOG);	//Mostrar Temperatura no display
		display.print(" ");
		display.setTextSize(1);
		display.cp437(true);                       //Desenhar "º" no display
		display.write(167);
		display.setTextSize(1.5);
		display.print("C");

		display.display();
		xSemaphoreGive(xMutex);                   //Mutex
#ifdef DEBUG_PRINT_ALL
		Serial.println(tempANALOG);
#endif
		vTaskDelayUntil(&xLastWakeTime, (150 / portTICK_PERIOD_MS)); //Tarefa ocorre durante 250ms
	}
}
void vTask_Temp(void *pvParameters) {    //Inicio Tarefa TempInLCD
	TickType_t xLastWakeTime;             //Variavel para determinar nº de ticks
	xLastWakeTime = xTaskGetTickCount();
	float oldtemp = 0;
	DHT dht(DHTPIN, DHTTYPE);                //Definir Sensor de Temperatura

	dht.begin();

	for (;;) {
		float temp = dht.readTemperature(); //Receber e ler temperatura do sensor DHT11
		if (isnan(temp)) {
#ifdef DEBUG_PRINT_ALL
			Serial.println("Failed to read from DHT sensor!");
#endif
			temp = oldtemp;
		} else {
			oldtemp = temp;
		}
		xQueueSendToBack(xQueueTemp, &temp, 0); //Enviar valor temp para a Brain

#ifdef DEBUG_PRINT_ALL
		Serial.print(F("Temperature: "));
		Serial.println(temp);
		Serial.println(F("Temperature: OK"));
#endif

		vTaskDelayUntil(&xLastWakeTime, (150 / portTICK_PERIOD_MS)); //Tarefa ocorre durante 250ms
	}
}                                              //Inicio Tarefa TempInLCD

void vTaskFAN(void *pvParameters) {       //Inicio Tarefa BUZER
	int INA = 14; //for ESP32
	int INB = 13; //for ESP32
	float TempBrain;
	pinMode(INA, OUTPUT);               //Definir pin do Fan
	pinMode(INB, OUTPUT);               //Definir pin do Fan

	for (;;) {

		digitalWrite(INA, HIGH); //turn Fan on
		digitalWrite(INB, LOW); //turn Fan on
		vTaskDelay(1000 / portTICK_PERIOD_MS);
#ifdef DEBUG_PRINT_1
		Serial.println(F("FAN: ON"));
#endif
		digitalWrite(INA, LOW);  //turn Fan off
		digitalWrite(INB, LOW);  //turn Fan off
		vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef DEBUG_PRINT_1
		Serial.println(F("FAN: OFF"));
#endif
#ifdef DEBUG_PRINT_ALL
			Serial.println(F("FAN: OK"));
#endif
		vTaskDelete(NULL);                //Eliminar tarefa Fan
	}
}										//FIM Tarefa FAN

void vTaskLED(void *pvParameters) {   //Inicio Tarefa LED
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	float TempBrain;
	int freq = 100000;               //Frequencia LED
	int ledChannel_Red = 5;            //PIN LED Vermelho
	int resolution = 10;
	static volatile int dutyCycle_Red =0;

	ledcSetup(ledChannel_Red, freq, resolution);
	ledcAttachPin(LED_PIN_Red, ledChannel_Red);

	for (;;) {
		xQueuePeek(xQueueTempBRAIN, &TempBrain, 0); //Receber valor de TempBRAIN
		if (TempBrain >= MAX_TEMP) {
#ifdef DEBUG_PRINT_1
			Serial.println(F("LED's TempBrain:"));
			Serial.println(TempBrain);
#endif
			for (dutyCycle_Red = 0; dutyCycle_Red <= pow(2, resolution);
					dutyCycle_Red++) {  //Piscar LED Vermelho
				ledcWrite(ledChannel_Red, dutyCycle_Red);
#ifdef DEBUG_PRINT_ALL
				Serial.print(F("LED's dutyCycle_Red:"));
				Serial.println(dutyCycle_Red);
				Serial.flush();
#endif
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			for (dutyCycle_Red = pow(2, resolution); dutyCycle_Red >= 0;
					dutyCycle_Red--) {
				ledcWrite(ledChannel_Red, dutyCycle_Red);
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
		}
#ifdef DEBUG_PRINT_ALL
		Serial.println(F("LED's: OK"));
#endif
	}
} //Fim Tarefa LED

void vTaskADC(void *pvParameters) {          //Inicio Tarefa Sensor Analógico
	char *pcTaskName;
	TickType_t xLastWakeTime;
	int analog_value = 0;
	float analog_voltage = 0;
	float temp_LM35 = 0;
	;

	analogReadResolution(ADC_RESOLUTION);
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		Serial.print(pcTaskName);
		analog_value = analogRead(ADC_1_4);
		analog_voltage = analog_value * (VREF_PLUS - VREF_MINUS)
				/ (pow(2.0, (float) ADC_RESOLUTION)) + VREF_MINUS;
		temp_LM35 = ((analog_voltage * 100));
#ifdef DEBUG_PRINT_ALL
			Serial.print("ADC_1_4: ");
			Serial.println(analog_value);
			Serial.print("ADC_1_4 VOLT: ");
			Serial.println(analog_voltage);
			Serial.print("LM35 DEG: ");
			Serial.println(temp_LM35);
#endif
#ifdef DEBUG_PRINT_ALL
		Serial.println(F("ADC's: OK"));
#endif
		xQueueSendToBack(xQueueADC, &temp_LM35, 0); //Enviar valor temp para a Brain
		vTaskDelayUntil(&xLastWakeTime, (150 / portTICK_PERIOD_MS)); //Tarefa ocorre durante 150ms
	}
}                                              //Fim da Tarefa ADC

void loop() {
	vTaskDelete( NULL);
}

