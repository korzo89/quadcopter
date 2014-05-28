/*
 * control.c
 *
 *  Created on: 28 maj 2014
 *      Author: Korzo
 */

#include "control.h"

#include <drivers/buzzer.h>
#include <modules/rcp.h>

//-----------------------------------------------------------------

static bool armed;
static control_t control;

static buzzer_step_t arm_seq[] = {
        { NOTE_F4, BUZZER_SEQ_WAIT(80) },
        { NOTE_A4, BUZZER_SEQ_WAIT(80) },
        { NOTE_C5, BUZZER_SEQ_WAIT(80) },
        { 0,       BUZZER_SEQ_STOP     }
};

static buzzer_step_t disarm_seq[] = {
        { NOTE_C5, BUZZER_SEQ_WAIT(80) },
        { NOTE_A4, BUZZER_SEQ_WAIT(80) },
        { NOTE_F4, BUZZER_SEQ_WAIT(80) },
        { 0,       BUZZER_SEQ_STOP     }
};

//-----------------------------------------------------------------

static void rcp_callback(rcp_message_t *msg)
{
    memcpy((uint8_t*)&control, msg->packet.data, sizeof(control_t));

    if (armed && !control.flags.sw1)
    {
        armed = false;
        buzzer_play_seq(disarm_seq);
    }
    else if (!armed && control.flags.sw1)
    {
        armed = true;
        buzzer_play_seq(arm_seq);
    }
}

//-----------------------------------------------------------------

result_t control_init(void)
{
    armed = false;

    rcp_register_callback(RCP_CMD_CONTROL, rcp_callback, false);

    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_current(control_t *out)
{
    memcpy(out, &control, sizeof(control_t));
    return RES_OK;
}
