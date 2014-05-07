/*
 * adc.c
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#include "adc.h"

#include <stellaris_config.h>

//-----------------------------------------------------------------

void adcConfig(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
}

//-----------------------------------------------------------------

unsigned long adcGetValue(void)
{
    ADCIntClear(ADC0_BASE, 0);
    ADCProcessorTrigger(ADC0_BASE, 0);
    while (!ADCIntStatus(ADC0_BASE, 0, false));
    ADCIntClear(ADC0_BASE, 0);

    unsigned long val;
    ADCSequenceDataGet(ADC0_BASE, 0, &val);

    return val;
}


