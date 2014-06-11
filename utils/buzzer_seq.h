/*
 * buzzer_seq.h
 *
 *  Created on: 12 cze 2014
 *      Author: Korzo
 */

#ifndef BUZZER_SEQ_H_
#define BUZZER_SEQ_H_

//-----------------------------------------------------------------

#include <defs.h>
#include <drivers/buzzer.h>

//-----------------------------------------------------------------

typedef enum
{
    BUZZER_SEQ_ARM,
    BUZZER_SEQ_DISARM,
    BUZZER_SEQ_CONNECTED,
    BUZZER_SEQ_LOST,
    BUZZER_SEQ_CONFIRM,
    BUZZER_SEQ_PRESS
} buzzer_seq_t;

//-----------------------------------------------------------------

buzzer_step_t* buzzer_seq_lib_get(buzzer_seq_t type);

result_t buzzer_seq_lib_play(buzzer_seq_t type);

//-----------------------------------------------------------------

#endif /* BUZZER_SEQ_H_ */
