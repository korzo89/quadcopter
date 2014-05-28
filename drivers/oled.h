#ifndef OLED_H_
#define OLED_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

#define OLED_COLS       128
#define OLED_ROWS       ( 64 / 8 )

//-----------------------------------------------------------------

bool oled_init(void);

void oled_clear(void);
void oled_clear_rect(uint8_t row, uint8_t col, uint8_t width, uint8_t height);
void oled_set_pos(uint8_t row, uint8_t col);

void oled_disp_str(char *str);
void oled_disp_str_at(char *str, uint8_t row, uint8_t col);
void oled_disp_char(char c);
void oled_disp_symbol(uint8_t *sym, uint8_t len);

bool oled_send_data(uint8_t data);
bool oled_send_cmd(uint8_t cmd);

//-----------------------------------------------------------------

#endif /* OLED_H_ */
