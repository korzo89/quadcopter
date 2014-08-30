/*
 * buzzer.h
 *
 *  Created on: 07-05-2014
 *      Author: Korzo
 */

#ifndef BUZZER_H_
#define BUZZER_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define SEQ_WAIT(x)  (x)
#define SEQ_LOOP     -1
#define SEQ_STOP     -2

//-----------------------------------------------------------------

struct buzzer_step
{
    uint32_t freq;
    int action;
};

//-----------------------------------------------------------------

void buzzer_init(void);

void buzzer_set_freq(uint32_t freq);

result_t buzzer_play_seq(const struct buzzer_step *seq);
result_t buzzer_stop_seq(void);

//-----------------------------------------------------------------

#endif /* BUZZER_H_ */
