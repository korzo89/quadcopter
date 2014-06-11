/*
 * buzzer_seq.c
 *
 *  Created on: 12 cze 2014
 *      Author: Korzo
 */

#include "buzzer_seq.h"

#include <utils/buzzer_notes.h>

//-----------------------------------------------------------------

static const buzzer_step_t SEQ_ARM[] = {
    { NOTE_F4, SEQ_WAIT(80) },
    { NOTE_A4, SEQ_WAIT(80) },
    { NOTE_C5, SEQ_WAIT(80) },
    { 0,       SEQ_STOP     }
};

static const buzzer_step_t SEQ_DISARM[] = {
    { NOTE_C5, SEQ_WAIT(80) },
    { NOTE_A4, SEQ_WAIT(80) },
    { NOTE_F4, SEQ_WAIT(80) },
    { 0,       SEQ_STOP     }
};

static const buzzer_step_t SEQ_CONNECTED[] = {
    { NOTE_D5, SEQ_WAIT(100) },
    { NOTE_G5, SEQ_WAIT(100) },
    { 0,       SEQ_STOP      }
};

static const buzzer_step_t SEQ_LOST[] = {
    { NOTE_G5, SEQ_WAIT(100) },
    { NOTE_D5, SEQ_WAIT(100) },
    { 0,       SEQ_STOP      }
};

static const buzzer_step_t SEQ_CONFIRM[] = {
    { NOTE_C5, SEQ_WAIT(30) },
    { NOTE_D5, SEQ_WAIT(30) },
    { 0,       SEQ_STOP     }
};

static const buzzer_step_t SEQ_PRESS[] = {
    { NOTE_C5, SEQ_WAIT(10) },
    { 0,       SEQ_STOP     }
};

const buzzer_step_t *SEQUENCES[] = {
    [BUZZER_SEQ_ARM]        = SEQ_ARM,
    [BUZZER_SEQ_DISARM]     = SEQ_DISARM,
    [BUZZER_SEQ_CONNECTED]  = SEQ_CONNECTED,
    [BUZZER_SEQ_LOST]       = SEQ_LOST,
    [BUZZER_SEQ_CONFIRM]    = SEQ_CONFIRM,
    [BUZZER_SEQ_PRESS]      = SEQ_PRESS
};

//-----------------------------------------------------------------

buzzer_step_t* buzzer_seq_lib_get(buzzer_seq_t type)
{
    return (buzzer_step_t*)SEQUENCES[(int)type];
}

//-----------------------------------------------------------------

result_t buzzer_seq_lib_play(buzzer_seq_t type)
{
    return buzzer_play_seq(buzzer_seq_lib_get(type));
}

