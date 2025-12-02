#ifndef LCD_I2C_H__
#define LCD_I2C_H__

#include "stm32f4xx_hal.h"

// LCD I2C ?? (0x27 ?? 0x3F? ???, ?? ????? ?? ??)
#define LCD_I2C_ADDR (0x27 << 1)  // 0x27 ?? 0x3F? ?? ??

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x00
#define LCD_RS        0x01

// ?? ??
void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_cmd(uint8_t cmd);           // ??? ?? ??
void lcd_send_data(uint8_t data);
void lcd_send_raw_cmd(uint8_t cmd);       // ??? ?? ??
void lcd_send_internal(uint8_t data, uint8_t flags);  // ??? ?? ??
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_create_char(uint8_t location, uint8_t charmap[]);

#endif // LCD_I2C_H__
