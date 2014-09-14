#include "oled.h"
#include "oled_font.h"
#include <drivers/ext_i2c.h>
#include <utils/delay.h>
#include <string.h>

//-----------------------------------------------------------------

#define OLED_ADDRESS    0x3C

//-----------------------------------------------------------------

struct oled_obj
{
    i2c_t *i2c_if;

    uint8_t curr_row;
    uint8_t curr_col;

    bool    present;
};

static struct oled_obj oled;

//-----------------------------------------------------------------

bool oled_init(void)
{
    memset(&oled, 0, sizeof(oled));

    oled.i2c_if = ext_i2c_get_if();

    if (!oled_send_cmd(0xAE))
        return false;
    oled_send_cmd(0x2E);
    oled_send_cmd(0xA4);
    DELAY_MS(5);
    oled_send_cmd(0xA1);
    oled_send_cmd(0xC8);
    oled_send_cmd(0xAF);
    oled_send_cmd(0x20);
    oled_send_cmd(0x02);
    oled_send_cmd(0xA6);
    DELAY_MS(5);
    oled_clear();

    oled.present = true;
    return true;
}

//-----------------------------------------------------------------

bool oled_is_present(void)
{
    return oled.present;
}

//-----------------------------------------------------------------

void oled_clear(void)
{
    if (!oled.present)
        return;

    int i, j;
    for (i = 0; i < 8; ++i)
    {
        oled_set_pos(i, 0);
        for (j = 0; j < 128; ++j)
            oled_send_data(0x00);
    }
    oled_set_pos(0, 0);
}

//-----------------------------------------------------------------

void oled_clear_rect(uint8_t row, uint8_t col, uint8_t width, uint8_t height)
{
    if (!oled.present)
        return;

    uint8_t temp_row = oled.curr_row;
    uint8_t temp_col = oled.curr_col;

    int i, j;
    for (i = 0; i < height; ++i)
    {
        oled_set_pos(row + i, col);
        for (j = 0; j < width; ++j)
            oled_send_data(0x00);
    }

    oled_set_pos(temp_row, temp_col);
}

//-----------------------------------------------------------------

void oled_set_pos(uint8_t row, uint8_t col)
{
    if (!oled.present)
        return;

    oled.curr_row = row % OLED_ROWS;
    oled.curr_col = col % OLED_COLS;

    oled_send_cmd(0xB0 + oled.curr_row);
    oled_send_cmd(0x00 + (8 * oled.curr_col & 0x0F));
    oled_send_cmd(0x10 + ((8 * oled.curr_col >> 4) & 0x0F));
}

//-----------------------------------------------------------------

void oled_disp_str(const char *str)
{
    if (!oled.present)
        return;

    char c;
    while (c = *str++)
        oled_disp_char(c);
}

//-----------------------------------------------------------------

void oled_disp_str_at(const char *str, uint8_t row, uint8_t col)
{
    if (!oled.present)
        return;

    oled_set_pos(row, col);
    oled_disp_str(str);
}

//-----------------------------------------------------------------

void oled_disp_char(char c)
{
    if (!oled.present)
        return;

    if (c == '\n')
    {
        oled_set_pos(oled.curr_row + 1, 0);
        return;
    }

    int i;
    for (i = 0; i < 8; ++i)
        oled_send_data(OLED_FONT[c - 0x20][i]);
}

//-----------------------------------------------------------------

void oled_disp_symbol(const uint8_t *sym, uint8_t len)
{
    if (!oled.present)
        return;

    while (len--)
        oled_send_data(*sym++);
}

//-----------------------------------------------------------------

bool oled_send_data(uint8_t data)
{
    if (i2c_write_reg(oled.i2c_if, OLED_ADDRESS, 0x40, &data, 1) == RES_OK)
    {
        oled.curr_col++;
        if (oled.curr_col == OLED_COLS)
            oled_set_pos(oled.curr_row + 1, 0);
        return true;
    }
    return false;
}

//-----------------------------------------------------------------

bool oled_send_cmd(uint8_t cmd)
{
    return i2c_write_reg(oled.i2c_if, OLED_ADDRESS, 0x80, &cmd, 1) == RES_OK;
}

