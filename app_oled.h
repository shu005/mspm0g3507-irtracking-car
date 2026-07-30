/*
 * app_oled.h
 *
 * SSD1306 128x64 OLED driver for MSPM0G3507.
 *
 * Hardware configuration used by this driver:
 *   I2C instance : I2C_0_INST
 *   SDA          : PA28
 *   SCL          : PA31
 *   bus speed    : 100 kHz
 *   7-bit address: auto-detect 0x3C, then 0x3D
 */

#ifndef APP_OLED_H_
#define APP_OLED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH                 128U
#define OLED_HEIGHT                 64U
#define OLED_PAGE_COUNT              8U

#define OLED_I2C_ADDRESS_1        0x3CU
#define OLED_I2C_ADDRESS_2        0x3DU

/*
 * Diagnostic variables:
 *   g_oled_address = 0x3C/0x3D after successful initialization
 *   g_oled_address = 0xFF when no OLED has acknowledged
 */
extern volatile uint8_t g_oled_address;
extern volatile uint32_t g_oled_i2c_error_count;

/*
 * Initialize an SSD1306 OLED.
 *
 * The function automatically tries 7-bit address 0x3C and then 0x3D.
 * It returns true only if an address acknowledges and initialization succeeds.
 */
bool OLED_Init(void);

/* Fill the local 128x64 framebuffer with 0x00 or 0xFF. */
void OLED_Clear(void);
void OLED_Fill(uint8_t pattern);

/* Send the complete framebuffer to the OLED. */
bool OLED_Refresh(void);

/*
 * Send only selected 8-pixel pages from the framebuffer.
 * first_page: 0..7
 * page_count: 1..8-first_page
 *
 * Updating only the timer pages keeps the I2C transfer short enough that
 * the line-following control loop is not noticeably disturbed.
 */
bool OLED_RefreshPages(uint8_t first_page, uint8_t page_count);

/*
 * Draw into the local framebuffer.
 * Coordinates use pixels: x=0..127, y=0..63.
 */
void OLED_DrawPixel(uint8_t x, uint8_t y, bool on);
void OLED_DrawRectangle(uint8_t x,
                        uint8_t y,
                        uint8_t width,
                        uint8_t height,
                        bool on);

/*
 * Draw 5x7 ASCII text into the local framebuffer.
 * page is 0..7; one text row occupies one 8-pixel page.
 * Lower-case letters are displayed as upper-case.
 */
void OLED_ShowChar(uint8_t x, uint8_t page, char character);
void OLED_ShowString(uint8_t x, uint8_t page, const char *text);

/* Runtime display controls. */
bool OLED_SetDisplayOn(bool on);
bool OLED_SetInverted(bool inverted);
bool OLED_SetContrast(uint8_t contrast);

/* Return the detected 7-bit address, or 0xFF if initialization failed. */
uint8_t OLED_GetAddress(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_OLED_H_ */
