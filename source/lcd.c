#include "lcd.h"

static I2C_HandleTypeDef *lcd_i2c = NULL;

void lcd_init(I2C_HandleTypeDef *hi2c) {
    lcd_i2c = hi2c;

    // LCD ??? ???
    HAL_Delay(100);

    // 4?? ??? ???? ?? ??? ???
    for (int i = 0; i < 3; i++) {
        lcd_send_raw_cmd(0x30);
        HAL_Delay(i == 0 ? 5 : 1);
    }

    lcd_send_raw_cmd(0x20); // 4-bit mode
    HAL_Delay(1);

    // ?? ??
    lcd_send_cmd(0x28); // Function set: 4-bit, 2-line, 5x8 dots
    lcd_send_cmd(0x08); // Display off
    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(2);
    lcd_send_cmd(0x06); // Entry mode set: increment, no shift
    lcd_send_cmd(0x0C); // Display on, cursor off, blink off
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_send_internal(cmd, 0);
}

void lcd_send_data(uint8_t data) {
    lcd_send_internal(data, LCD_RS);
}

void lcd_send_raw_cmd(uint8_t cmd) {
    uint8_t high_nibble = cmd & 0xF0;
    uint8_t buf[2];

    buf[0] = high_nibble | LCD_BACKLIGHT | LCD_ENABLE;
    buf[1] = high_nibble | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(lcd_i2c, LCD_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
    HAL_Delay(1); // Enable pulse width
}

void lcd_send_internal(uint8_t data, uint8_t flags) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;
    uint8_t buf[4];

    // High nibble
    buf[0] = high_nibble | flags | LCD_BACKLIGHT | LCD_ENABLE;
    buf[1] = high_nibble | flags | LCD_BACKLIGHT;

    // Low nibble
    buf[2] = low_nibble | flags | LCD_BACKLIGHT | LCD_ENABLE;
    buf[3] = low_nibble | flags | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(lcd_i2c, LCD_I2C_ADDR, buf, 4, HAL_MAX_DELAY);
    HAL_Delay(1); // Command execution time
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2); // Clear ??? ?? ??? ??? 2ms ??
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54}; // 4? LCD ??
    if (row < 4) { // ?? ??
        lcd_send_cmd(0x80 | (col + row_offsets[row]));
    }
}

void lcd_create_char(uint8_t location, uint8_t charmap[]) {
    location &= 0x07; // 0~7 ??? ??
    lcd_send_cmd(0x40 | (location << 3)); // CGRAM ?? ??
    for (int i = 0; i < 8; i++) {
        lcd_send_data(charmap[i]);
    }
    // ?? DDRAM?? ????
    lcd_set_cursor(0, 0);
}
