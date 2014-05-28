/*
 * control.c
 *
 *  Created on: 28 maj 2014
 *      Author: Korzo
 */

#include "control.h"

#include <drivers/buzzer.h>
#include <modules/rcp.h>
#include <utils/delay.h>

//-----------------------------------------------------------------

static bool armed;
static bool connected;
static control_t control;

static const buzzer_step_t SEQ_ARM[] = {
        { NOTE_F4, BUZZER_SEQ_WAIT(80) },
        { NOTE_A4, BUZZER_SEQ_WAIT(80) },
        { NOTE_C5, BUZZER_SEQ_WAIT(80) },
        { 0,       BUZZER_SEQ_STOP     }
};

static const buzzer_step_t SEQ_DISARM[] = {
        { NOTE_C5, BUZZER_SEQ_WAIT(80) },
        { NOTE_A4, BUZZER_SEQ_WAIT(80) },
        { NOTE_F4, BUZZER_SEQ_WAIT(80) },
        { 0,       BUZZER_SEQ_STOP     }
};

static const buzzer_step_t SEQ_CONNECTED[] = {
        { NOTE_D5, BUZZER_SEQ_WAIT(100) },
        { NOTE_G5, BUZZER_SEQ_WAIT(100) },
        { 0,       BUZZER_SEQ_STOP      }
};

static const buzzer_step_t SEQ_LOST[] = {
        { NOTE_G5, BUZZER_SEQ_WAIT(100) },
        { NOTE_D5, BUZZER_SEQ_WAIT(100) },
        { 0,       BUZZER_SEQ_STOP      }
};

//-----------------------------------------------------------------

static void rcp_callback(rcp_message_t *msg)
{
    memcpy((uint8_t*)&control, msg->packet.data, sizeof(control_t));

    if (armed && !control.flags.sw1)
    {
        armed = false;
        buzzer_play_seq((buzzer_step_t*)SEQ_DISARM);
    }
    else if (!armed && control.flags.sw1)
    {
        armed = true;
        buzzer_play_seq((buzzer_step_t*)SEQ_ARM);
    }
}

//-----------------------------------------------------------------

static void control_task(void *params)
{
    while (1)
    {
        if (rcp_is_connected())
        {
            if (!connected)
            {
                buzzer_play_seq((buzzer_step_t*)SEQ_CONNECTED);
                connected = true;
            }
        }
        else if (connected)
        {
            buzzer_play_seq((buzzer_step_t*)SEQ_LOST);
            connected = false;
        }

        DELAY_MS(100);
    }
}

//-----------------------------------------------------------------

result_t control_init(void)
{
    armed = false;
    connected = false;

    rcp_register_callback(RCP_CMD_CONTROL, rcp_callback, false);

    if (xTaskCreate(control_task, (signed portCHAR*)"CTRL",
            configMINIMAL_STACK_SIZE, NULL, 2, NULL) != pdPASS)
        return RES_ERR_FATAL;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_current(control_t *out)
{
    memcpy(out, &control, sizeof(control_t));
    return RES_OK;
}
