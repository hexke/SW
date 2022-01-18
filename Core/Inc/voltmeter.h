#ifndef VOLTMETER_H
#define VOLTMETER_H

#include "main.h"
#include "cmsis_os.h"

#include "usbd_cdc_if.h"

void Voltmeter_init(void);
void vTerminal(void  * argument);
void vADC(void  * argument);

#endif//VOLTMETER_H
