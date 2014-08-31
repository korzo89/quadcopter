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

enum buzzer_mode
{
    BUZZER_MODE_FORCE,
    BUZZER_MODE_IGNORE,
    BUZZER_MODE_QUEUE
};

//-----------------------------------------------------------------

void buzzer_init(void);

void buzzer_set_freq(uint32_t freq);

bool buzzer_play_seq(const struct buzzer_step *seq, enum buzzer_mode mode);
void buzzer_stop(bool clear_queue);

//-----------------------------------------------------------------

#endif /* BUZZER_H_ */
