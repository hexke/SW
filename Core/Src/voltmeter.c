/*
 * voltmeter.c
 *
 *  Created on: 23 gru 2021
 *      Author: rczer
 */
#include "voltmeter.h"

#define IDLE_TaskPriority 0

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim6;

const uint32_t AvailableADC[] = {
		(uint32_t) &hadc1,
		(uint32_t) &hadc2,
		(uint32_t) &hadc3};


volatile xSemaphoreHandle Delay_Sem;

 xQueueHandle Term_Queue;
 volatile xQueueHandle ADC_Queue;


 /**
  * @brief funkcja Voltometer_init inicjalizuje semafory, kolejki oraz zadania potrzebne systemowi operacyjnemu FreeRTOS
  * do poprawnego działania.
  */
void Voltmeter_init(void){

	/** wykorzystywany w przerwaniu (podczas trybu continous) sygnalizując, że minęła kolejna sekunda.*/
	Delay_Sem = xSemaphoreCreateBinary();

	Term_Queue = xQueueCreate( 2, (unsigned portBASE_TYPE) sizeof(uint16_t));
	ADC_Queue = xQueueCreate( 2, (unsigned portBASE_TYPE) sizeof(uint8_t));

	xTaskCreate(vTerminal,"Terminal",configMINIMAL_STACK_SIZE,NULL,IDLE_TaskPriority + 1,NULL);
	xTaskCreate(vADC,"ADC",configMINIMAL_STACK_SIZE,NULL,IDLE_TaskPriority + 1,NULL);
}

/**
 * @brief funkcja vTerminal wykorzystywana jest przez system operacyjny FreeRTOS.
 * 	Spełnia zadanie komunikacji pomiędzy płytką a komputerem poprzez port szeregowy USART
 * @param argument
 */
void vTerminal(void  * argument)
{
uint8_t i;
uint8_t voltage[10] = "0.000 V\n\r";
uint16_t tmp =0;

  for(;;)
  {

	if(xQueueReceive(Term_Queue,&tmp,0) == pdTRUE){
			tmp = 256*tmp/100/4096;
			for(i =4;i<0;i--){
			if(i ==1 ) continue;
			voltage[i] = '0' + tmp%10;
			tmp/=10;
		}

	CDC_Transmit_HS((uint8_t*)voltage,9);
  }
	vTaskDelay(100/portTICK_RATE_MS);
  }
}


/**
 * @brief funkcja vADC przetwarza przesłane przez port komendy a następnie odpowiednio na nie reaguje.
 * Odpowiada za pobranie wartości przetworzonego napięcia z przetworników, przełączanie między kanałami jak i ich tryby pracy
 * dostępne są dwa tryby pracy:
 * s - pojedynczy pomiar napięcia
 * c - pomiar napięcia co 1s
 * @param argument
 */
void vADC(void  * argument)
{

	uint16_t result;
	uint8_t instr;
	uint8_t zmiana_ADC = 0;
	uint8_t continous = 0;
	uint8_t currentADC = 1;
	HAL_ADC_Start((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);

  for(;;)
  {

	if( xQueueReceive(ADC_Queue,&instr,1) == pdTRUE){
		HAL_TIM_Base_Stop_IT(&htim6);
		continous = 0;
		switch (instr){
			case 'C':
			case 'c':{
				HAL_TIM_Base_Start_IT(&htim6);
				continous = 1;
				break;
			}
			case 'S':
			case 's':{
				result = HAL_ADC_GetValue((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);
				xQueueSendToFront(Term_Queue, &result,5);
				break;
			}
			case 'I':
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

	if(continous == 1 && xSemaphoreTake(Delay_Sem,0) == pdTRUE){
		result = HAL_ADC_GetValue((ADC_HandleTypeDef *) AvailableADC[currentADC-1]);
		xQueueSendToBack(Term_Queue, &result,5);
		}
	vTaskDelay(10/portTICK_RATE_MS);
  }
}


