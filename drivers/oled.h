#ifndef OLED_H_
#define OLED_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

#define OLED_COLS       128
#define OLED_ROWS       ( 64 / 8 )

//-----------------------------------------------------------------

bool oledInit(void);

void oledClear(void);
void oledSetPos(uint8_t row, uint8_t col);

void oledDispStr(char *str);
void oledDispStrAt(char *str, uint8_t row, uint8_t col);
void oledDispChar(char c);

bool oledSendData(uint8_t data);
bool oledSendCmd(uint8_t cmd);

//-----------------------------------------------------------------

#endif /* OLED_H_ */
