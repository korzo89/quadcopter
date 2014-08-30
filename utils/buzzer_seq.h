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

enum buzzer_seq
{
    BUZZER_SEQ_ARM,
    BUZZER_SEQ_DISARM,
    BUZZER_SEQ_CONNECTED,
    BUZZER_SEQ_LOST,
    BUZZER_SEQ_CONFIRM,
    BUZZER_SEQ_PRESS
};

//-----------------------------------------------------------------

const struct buzzer_step* buzzer_seq_lib_get(enum buzzer_seq type);

result_t buzzer_seq_lib_play(enum buzzer_seq type);

//-----------------------------------------------------------------

#endif /* BUZZER_SEQ_H_ */
