#ifndef VOLTMETER_H
#define VOLTMETER_H

#include "main.h"
#include "cmsis_os.h"

#include "usbd_cdc_if.h"
#include <stdio.h>

void Voltmeter_init(void);
void vStartTerminal(void  * argument);
void vStartADC(void  * argument);

#endif//VOLTMETER_H
