/*
 * voltmeter.c
 *
 *  Created on: 23 gru 2021
 *      Author: rczer
 */

/*!
 * plik voltometer.c przechowuje funkcje związane z:
 *  - komunikacją przez terminal szeregowy USART
 *  - obsługiwaniem przetworników analogowo-cyfrowych do pomiaru napięć na wybranym kanale
 *
 *  dostępne dla użytkownika komendy:
 * @param c ustawia tryb continous przetwornika,
 * @param s dokonuje pomiaru na aktualnie aktywnym kanale przetwornika
 * @param i określa chęć zmiany kanału na inny. Poprzedza liczbową wartość kanału.
 * @param numerKanału poprzedza ją komenda "i". Numer kanału jest wartością liczbową 1,2 lub 3 określajacą, który kanał ma być aktywny
 */

#include "voltmeter.h"

#define IDLE_TaskPriority 0

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/**
 * dostępne kanały przetwornika, między którymi można się przełączać
 */
const uint32_t AvailableADC[] = {
		(uint32_t) &hadc1,
		(uint32_t) &hadc2,
		(uint32_t) &hadc3};

TaskHandle_t* Terminal_CommHandle;
TaskHandle_t* ADC_CommHandle;

xQueueHandle Term_Queue;
xQueueHandle ADC_Queue;
xSemaphoreHandle Term_Sem;
xSemaphoreHandle ADC_Sem;

uint8_t ReceivedDataFlag = 0; /*!< flaga kontrolująca czy pojawiła się nowa komenda */
uint8_t ReceivedData; /*!< zmienna przechowująca przysłaną komendę */

/**
 * @brief Tworzy semafory oraz zadania potrzebne systemu operacyjnemu FreeRTOS
 */
void Voltmeter_init(void){

	vSemaphoreCreateBinary(Term_Sem);
	vSemaphoreCreateBinary(ADC_Sem);

	Term_Queue = xQueueCreate( 1, (unsigned portBASE_TYPE) sizeof(uint16_t));
	ADC_Queue = xQueueCreate( 1, (unsigned portBASE_TYPE) sizeof(char));

	xTaskCreate(vStartTerminal,"Terminal",configMINIMAL_STACK_SIZE,NULL,IDLE_TaskPriority + 1,Terminal_CommHandle);
	xTaskCreate(vStartADC,"ADC",configMINIMAL_STACK_SIZE,NULL,IDLE_TaskPriority + 1,ADC_CommHandle);
}


/**
 * @brief Służy do odbierania komend oraz wysyłania wyników pomiarów przetwornika
 * @param argument
 */
void vStartTerminal(void  * argument)
{

char voltage[9] = "0.000 V\n\r"; /*!< tablica przechowująca wynik ostatnio zmierzonej wartości napięcia */

uint8_t len; /*! zmienna pomocnicza przechowująca długość odbieranych danych */
uint16_t tmp =0; /*! zmienna pomocnicza za pomocą której dokonuje się konwersji napięcia na ciąg znaków */

  for(;;)
  {
	  if(ReceivedDataFlag == 1){
	  	ReceivedDataFlag = 0;
	  	len = sprintf((char*)str,"%c\n\r",(char)ReceivedData);
	  	xQueueSendToFront(ADC_Queue, &ReceivedData, 5);
	  	CDC_Transmit_HS((uint8_t*)str,len);
	  	xSemaphoreGive(ADC_Sem);
	  }

	if(xSemaphoreTake(Term_Sem,0)){
	xQueueReceive(Term_Queue,&tmp,0);

	voltage[4] = '0' + tmp%10;
	tmp/=10;
	voltage[3] = '0' + tmp%10;
	tmp/=10;
	voltage[2] = '0' + tmp%10;
	tmp/=10;
	voltage[1] = '.';

	voltage[0] = '0' + tmp%10;

	CDC_Transmit_HS((uint8_t*)voltage,9);
  }

	vTaskDelay(100);
  }
}

/**
 * @brief Funkcja odpowiedzialna za obsługę przetowrnika. Zmienia kanały przetwornika oraz pobiera przetowrzone napięcie.
 * Obsługuje tryby pojedynczego pomiaru jak i tzw. trybu "continous". Tryb continous zwraca wartość napięcia co określoną jednostkę czasu.
 *
 * @param argument
 */
void vStartADC(void  * argument)
{

	uint16_t result;
	char instr;
	uint8_t zmiana_ADC = 0;
	uint8_t continous = 0;
	uint8_t currentADC = 1;

  for(;;)
  {
		HAL_ADC_Start((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);

	if( xSemaphoreTake(ADC_Sem, 0)){
		xQueueReceive(ADC_Queue,&instr,1);
		continous = 0;
		switch (instr){
			case 'c':{
				continous = 1;
				break;
			}

			case 's':{
				result = (2.56* (4096/HAL_ADC_GetValue((ADC_HandleTypeDef *) AvailableADC[currentADC-1])));
				xQueueSendToFront(Term_Queue, &result,5);
				xSemaphoreGive(Term_Sem);
				break;
			}
			case 'i':{
				zmiana_ADC = 1;
				break;
			}
			case '1':
			case '2':
			case '3':{
				if (zmiana_ADC == 1){
					HAL_ADC_Stop((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);
					zmiana_ADC = 0;
					currentADC = instr - '0';
					HAL_ADC_Start((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);
				}
			}
			default:{
				break;
			}
		}
	}

	if(continous == 1){
		result =(2.56* (4096/HAL_ADC_GetValue((ADC_HandleTypeDef *) AvailableADC[currentADC-1])));
		xQueueSendToBack(Term_Queue, &result,5);
		xSemaphoreGive(Term_Sem);
		vTaskDelay(990);
		}
	vTaskDelay(10);
  }
}
