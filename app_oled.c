/*
 * app_oled.c
 *
 * SSD1306 128x64 OLED driver for MSPM0G3507 DriverLib.
 *
 * Notes:
 *   1. The I2C address passed to DriverLib is a 7-bit address (0x3C/0x3D),
 *      not the 8-bit address 0x78/0x7A sometimes printed by vendors.
 *   2. This driver follows the vendor's SSD1306 initialization sequence.
 *   3. This driver uses short polling transfers and does not require I2C
 *      interrupts or DMA.
 */

#include "app_oled.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

#define OLED_CONTROL_COMMAND             0x00U
#define OLED_CONTROL_DATA                0x40U
#define OLED_SSD1306_COLUMN_OFFSET          0U

/*
 * The MSPM0 I2C controller FIFO accepts this complete packet:
 * one control byte followed by up to seven display bytes.
 */
#define OLED_I2C_DATA_BYTES_PER_PACKET      7U
#define OLED_I2C_PACKET_SIZE                8U

/*
 * This is a software polling limit, not a millisecond value.
 * At 32 MHz it is intentionally much longer than one 100 kHz I2C packet.
 */
#define OLED_I2C_TIMEOUT_COUNT          200000U
#define OLED_I2C_START_DELAY_CYCLES         64U

volatile uint8_t g_oled_address = 0xFFU;
volatile uint32_t g_oled_i2c_error_count = 0U;

static uint8_t g_oled_framebuffer[OLED_WIDTH * OLED_PAGE_COUNT];

static bool OLED_I2C_WaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_COUNT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(I2C_0_INST);

        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U)
        {
            return false;
        }

        if (((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) &&
            ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U))
        {
            return true;
        }

        timeout--;
    }

    return false;
}

static void OLED_I2C_Recover(void)
{
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    g_oled_i2c_error_count++;
}

static bool OLED_I2C_WritePacket(const uint8_t *data, uint32_t length)
{
    uint32_t filled;

    if ((data == NULL) || (length == 0U) ||
        (length > OLED_I2C_PACKET_SIZE) ||
        (g_oled_address > 0x7FU))
    {
        return false;
    }

    if (!OLED_I2C_WaitIdle())
    {
        OLED_I2C_Recover();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    filled = DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, length);

    if (filled != length)
    {
        OLED_I2C_Recover();
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   (uint32_t)g_oled_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   length);

    /*
     * Give the controller enough cycles to leave its pre-transfer idle state
     * before polling for completion.
     */
    delay_cycles(OLED_I2C_START_DELAY_CYCLES);

    if (!OLED_I2C_WaitIdle())
    {
        OLED_I2C_Recover();
        return false;
    }

    return true;
}

static bool OLED_WriteCommand(uint8_t command)
{
    uint8_t packet[2];

    packet[0] = OLED_CONTROL_COMMAND;
    packet[1] = command;

    return OLED_I2C_WritePacket(packet, 2U);
}

static bool OLED_WriteCommands(const uint8_t *commands, uint32_t count)
{
    uint32_t index;

    if (commands == NULL)
    {
        return false;
    }

    for (index = 0U; index < count; index++)
    {
        if (!OLED_WriteCommand(commands[index]))
        {
            return false;
        }
    }

    return true;
}

static bool OLED_WriteData(const uint8_t *data, uint32_t count)
{
    uint8_t packet[OLED_I2C_PACKET_SIZE];
    uint32_t sent = 0U;

    if (data == NULL)
    {
        return false;
    }

    packet[0] = OLED_CONTROL_DATA;

    while (sent < count)
    {
        uint32_t index;
        uint32_t chunk = count - sent;

        if (chunk > OLED_I2C_DATA_BYTES_PER_PACKET)
        {
            chunk = OLED_I2C_DATA_BYTES_PER_PACKET;
        }

        for (index = 0U; index < chunk; index++)
        {
            packet[index + 1U] = data[sent + index];
        }

        if (!OLED_I2C_WritePacket(packet, chunk + 1U))
        {
            return false;
        }

        sent += chunk;
    }

    return true;
}

static bool OLED_InitializeAtAddress(uint8_t address)
{
    /*
     * SSD1306 128x64 initialization translated from the official
     * STC89C52RC software-I2C example supplied with this display.
     */
    static const uint8_t init_commands[] = {
        0xAEU,
        0x00U,
        0x10U,
        0x40U,
        0x81U, 0xCFU,
        0xA1U,
        0xC8U,
        0xA6U,
        0xA8U, 0x3FU,
        0xD3U, 0x00U,
        0xD5U, 0x80U,
        0xD9U, 0xF1U,
        0xDAU, 0x12U,
        0xDBU, 0x40U,
        0x20U, 0x02U,
        0x8DU, 0x14U,
        0xA4U,
        0xA6U,
        0xAFU
    };

    g_oled_address = address;

    if (!OLED_WriteCommands(init_commands,
                            sizeof(init_commands) / sizeof(init_commands[0])))
    {
        return false;
    }

    return true;
}

bool OLED_Init(void)
{
    g_oled_address = OLED_I2C_ADDRESS_1;

    if (!OLED_InitializeAtAddress(OLED_I2C_ADDRESS_1))
    {
        OLED_I2C_Recover();
        g_oled_address = OLED_I2C_ADDRESS_2;

        if (!OLED_InitializeAtAddress(OLED_I2C_ADDRESS_2))
        {
            OLED_I2C_Recover();
            g_oled_address = 0xFFU;
            return false;
        }
    }

    OLED_Clear();

    if (!OLED_Refresh())
    {
        g_oled_address = 0xFFU;
        return false;
    }

    return true;
}

void OLED_Clear(void)
{
    memset(g_oled_framebuffer, 0, sizeof(g_oled_framebuffer));
}

void OLED_Fill(uint8_t pattern)
{
    memset(g_oled_framebuffer, pattern, sizeof(g_oled_framebuffer));
}

bool OLED_RefreshPages(uint8_t first_page, uint8_t page_count)
{
    uint8_t page;
    uint8_t end_page;

    if ((g_oled_address > 0x7FU) ||
        (first_page >= OLED_PAGE_COUNT) ||
        (page_count == 0U) ||
        (page_count > (OLED_PAGE_COUNT - first_page)))
    {
        return false;
    }

    end_page = (uint8_t)(first_page + page_count);

    for (page = first_page; page < end_page; page++)
    {
        uint8_t column = OLED_SSD1306_COLUMN_OFFSET;
        uint8_t page_commands[3];

        page_commands[0] = (uint8_t)(0xB0U + page);
        page_commands[1] = (uint8_t)(0x00U | (column & 0x0FU));
        page_commands[2] = (uint8_t)(0x10U | ((column >> 4U) & 0x0FU));

        if (!OLED_WriteCommands(page_commands, 3U))
        {
            return false;
        }

        if (!OLED_WriteData(&g_oled_framebuffer[(uint32_t)page * OLED_WIDTH],
                            OLED_WIDTH))
        {
            return false;
        }
    }

    return true;
}

bool OLED_Refresh(void)
{
    return OLED_RefreshPages(0U, OLED_PAGE_COUNT);
}

void OLED_DrawPixel(uint8_t x, uint8_t y, bool on)
{
    uint32_t index;
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    index = ((uint32_t)(y >> 3U) * OLED_WIDTH) + x;
    mask = (uint8_t)(1U << (y & 0x07U));

    if (on)
    {
        g_oled_framebuffer[index] |= mask;
    }
    else
    {
        g_oled_framebuffer[index] &= (uint8_t)(~mask);
    }
}

void OLED_DrawRectangle(uint8_t x,
                        uint8_t y,
                        uint8_t width,
                        uint8_t height,
                        bool on)
{
    uint16_t right;
    uint16_t bottom;
    uint16_t position;

    if ((width == 0U) || (height == 0U) ||
        (x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    right = (uint16_t)x + width - 1U;
    bottom = (uint16_t)y + height - 1U;

    if (right >= OLED_WIDTH)
    {
        right = OLED_WIDTH - 1U;
    }

    if (bottom >= OLED_HEIGHT)
    {
        bottom = OLED_HEIGHT - 1U;
    }

    for (position = x; position <= right; position++)
    {
        OLED_DrawPixel((uint8_t)position, y, on);
        OLED_DrawPixel((uint8_t)position, (uint8_t)bottom, on);
    }

    for (position = y; position <= bottom; position++)
    {
        OLED_DrawPixel(x, (uint8_t)position, on);
        OLED_DrawPixel((uint8_t)right, (uint8_t)position, on);
    }
}

static void OLED_GetGlyph(char character, uint8_t glyph[5])
{
    const uint8_t *source;

    static const uint8_t glyph_space[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t glyph_question[5] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
    static const uint8_t glyph_period[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t glyph_colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t glyph_hyphen[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t glyph_slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};

    static const uint8_t digits[10][5] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
    };

    static const uint8_t letters[26][5] = {
        {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
        {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
        {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
        {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
        {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
        {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
        {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
        {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
        {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
        {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
        {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
        {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
        {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
        {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
        {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
        {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
        {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
        {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
        {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
        {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
    };

    if ((character >= 'a') && (character <= 'z'))
    {
        character = (char)(character - ('a' - 'A'));
    }

    if ((character >= '0') && (character <= '9'))
    {
        source = digits[(uint8_t)(character - '0')];
    }
    else if ((character >= 'A') && (character <= 'Z'))
    {
        source = letters[(uint8_t)(character - 'A')];
    }
    else
    {
        switch (character)
        {
            case ' ':
                source = glyph_space;
                break;
            case '.':
                source = glyph_period;
                break;
            case ':':
                source = glyph_colon;
                break;
            case '-':
                source = glyph_hyphen;
                break;
            case '/':
                source = glyph_slash;
                break;
            default:
                source = glyph_question;
                break;
        }
    }

    memcpy(glyph, source, 5U);
}

void OLED_ShowChar(uint8_t x, uint8_t page, char character)
{
    uint8_t glyph[5];
    uint8_t column;
    uint32_t index;

    if ((x >= OLED_WIDTH) || (page >= OLED_PAGE_COUNT))
    {
        return;
    }

    OLED_GetGlyph(character, glyph);
    index = ((uint32_t)page * OLED_WIDTH) + x;

    for (column = 0U; (column < 5U) && ((x + column) < OLED_WIDTH); column++)
    {
        g_oled_framebuffer[index + column] = glyph[column];
    }

    if ((x + 5U) < OLED_WIDTH)
    {
        g_oled_framebuffer[index + 5U] = 0x00U;
    }
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while ((*text != '\0') && (page < OLED_PAGE_COUNT))
    {
        if (x > (OLED_WIDTH - 6U))
        {
            x = 0U;
            page++;

            if (page >= OLED_PAGE_COUNT)
            {
                break;
            }
        }

        OLED_ShowChar(x, page, *text);
        x = (uint8_t)(x + 6U);
        text++;
    }
}

bool OLED_SetDisplayOn(bool on)
{
    return OLED_WriteCommand(on ? 0xAFU : 0xAEU);
}

bool OLED_SetInverted(bool inverted)
{
    return OLED_WriteCommand(inverted ? 0xA7U : 0xA6U);
}

bool OLED_SetContrast(uint8_t contrast)
{
    if (!OLED_WriteCommand(0x81U))
    {
        return false;
    }

    return OLED_WriteCommand(contrast);
}

uint8_t OLED_GetAddress(void)
{
    return g_oled_address;
}
