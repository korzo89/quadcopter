/*
 * buzzer_seq.c
 *
 *  Created on: 12 cze 2014
 *      Author: Korzo
 */

#include "buzzer_seq.h"

#include <utils/buzzer_notes.h>

//-----------------------------------------------------------------

static const struct buzzer_step seq_arm[] = {
    { NOTE_F4, SEQ_WAIT(80) },
    { NOTE_A4, SEQ_WAIT(80) },
    { NOTE_C5, SEQ_WAIT(80) },
    { 0,       SEQ_STOP     }
};

static const struct buzzer_step seq_disarm[] = {
    { NOTE_C5, SEQ_WAIT(80) },
    { NOTE_A4, SEQ_WAIT(80) },
    { NOTE_F4, SEQ_WAIT(80) },
    { 0,       SEQ_STOP     }
};

static const struct buzzer_step seq_connected[] = {
    { NOTE_D5, SEQ_WAIT(100) },
    { NOTE_G5, SEQ_WAIT(100) },
    { 0,       SEQ_STOP      }
};

static const struct buzzer_step seq_lost[] = {
    { NOTE_G5, SEQ_WAIT(100) },
    { NOTE_D5, SEQ_WAIT(100) },
    { 0,       SEQ_STOP      }
};

static const struct buzzer_step seq_confirm[] = {
    { NOTE_C5, SEQ_WAIT(30) },
    { NOTE_D5, SEQ_WAIT(30) },
    { 0,       SEQ_STOP     }
};

static const struct buzzer_step seq_error[] = {
    { NOTE_C4, SEQ_WAIT(80) },
    { 0,       SEQ_WAIT(30) },
    { NOTE_C4, SEQ_WAIT(80) },
    { 0,       SEQ_STOP     }
};

static const struct buzzer_step seq_press[] = {
    { NOTE_C5, SEQ_WAIT(10) },
    { 0,       SEQ_STOP     }
};

static const struct buzzer_step seq_axis_disabled[] = {
    { NOTE_C5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(80)  },
    { NOTE_C5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(200) },
    { 0,       SEQ_STOP      }
};

static const struct buzzer_step seq_axis_angle[] = {
    { NOTE_C5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(80)  },
    { NOTE_E5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(200) },
    { 0,       SEQ_STOP      }
};

static const struct buzzer_step seq_axis_rate[] = {
    { NOTE_E5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(80)  },
    { NOTE_E5, SEQ_WAIT(80)  },
    { 0,       SEQ_WAIT(200) },
    { 0,       SEQ_STOP      }
};

const struct buzzer_step *sequences[] = {
    [BUZZER_SEQ_ARM]            = seq_arm,
    [BUZZER_SEQ_DISARM]         = seq_disarm,
    [BUZZER_SEQ_CONNECTED]      = seq_connected,
    [BUZZER_SEQ_LOST]           = seq_lost,
    [BUZZER_SEQ_CONFIRM]        = seq_confirm,
    [BUZZER_SEQ_ERROR]          = seq_error,
    [BUZZER_SEQ_PRESS]          = seq_press,
    [BUZZER_SEQ_AXIS_DISABLED]  = seq_axis_disabled,
    [BUZZER_SEQ_AXIS_ANGLE]     = seq_axis_angle,
    [BUZZER_SEQ_AXIS_RATE]      = seq_axis_rate
};

//-----------------------------------------------------------------

const struct buzzer_step* buzzer_seq_lib_get(enum buzzer_seq type)
{
    return sequences[(uint32_t)type];
}

//-----------------------------------------------------------------

bool buzzer_seq_lib_play(enum buzzer_seq type, enum buzzer_mode mode)
{
    return buzzer_play_seq(buzzer_seq_lib_get(type), mode);
}

