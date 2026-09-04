#include "oled.h"
#include "i2c.h"

#include <string.h>

static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8U];
static uint8_t oled_column;
static uint8_t oled_page;
static uint8_t oled_ready;

static void Oled_BusRecover(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;

  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1U);
  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  /* 人工产生 STOP：SDA 低电平时释放 SCL，再释放 SDA。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1U);

  MX_I2C1_Init();
}

static HAL_StatusTypeDef Oled_Command(uint8_t command)
{
  uint8_t data[2] = {0x00U, command};
  return HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, data, 2U, 100U);
}

static void Oled_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
  uint16_t index;
  if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  index = (uint16_t)(y / 8U) * OLED_WIDTH + x;
  if (on != 0U)
  {
    oled_buffer[index] |= (uint8_t)(1U << (y % 8U));
  }
  else
  {
    oled_buffer[index] &= (uint8_t)~(1U << (y % 8U));
  }
}

static const uint8_t *Oled_Glyph(char ch)
{
  static const uint8_t blank[5] = {0, 0, 0, 0, 0};
  static const uint8_t colon[5] = {0, 0x36, 0x36, 0, 0};
  static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0};
  static const uint8_t digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
  };
  static const uint8_t letters[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
  };
  if (ch >= '0' && ch <= '9') return digits[(uint8_t)(ch - '0')];
  if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
  if (ch >= 'A' && ch <= 'Z') return letters[(uint8_t)(ch - 'A')];
  if (ch == ':') return colon;
  if (ch == '-') return dash;
  return blank;
}

HAL_StatusTypeDef Oled_Init(void)
{
  static const uint8_t init_commands[] = {
#if OLED_CONTROLLER_SH1106
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0xAD, 0x8B, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x80,
    0xD9, 0x1F, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
#else
    0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xD3, 0x00,
    0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40,
    0x8D, 0x14, 0xAF
#endif
  };
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t attempt;
  uint8_t i;

  oled_ready = 0U;
  for (attempt = 0U; attempt < 3U; attempt++)
  {
    Oled_BusRecover();
    HAL_Delay(20U);
    status = HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDRESS, 3U, 100U);
    if (status != HAL_OK)
    {
      continue;
    }

    for (i = 0U; i < sizeof(init_commands); i++)
    {
      status = Oled_Command(init_commands[i]);
      if (status != HAL_OK) break;
    }
    if (status == HAL_OK)
    {
      oled_ready = 1U;
      break;
    }
  }

  Oled_Clear();
  oled_column = 0U;
  oled_page = 0U;
  return status;
}

void Oled_Clear(void) { memset(oled_buffer, 0, sizeof(oled_buffer)); }

void Oled_SetCursor(uint8_t column, uint8_t page)
{
  /* 放大字库：字符宽 10 像素，间隔 1 像素；高度 14 像素。 */
  oled_column = (column < 11U) ? column : 0U;
  oled_page = (page < 7U) ? page : 0U;
}

void Oled_WriteChar(char ch)
{
  const uint8_t *glyph = Oled_Glyph(ch);
  uint8_t x;
  uint8_t y;
  uint8_t base_x = (uint8_t)(oled_column * 11U);
  uint8_t base_y = (uint8_t)(oled_page * 8U);

  if (oled_column >= 11U || oled_page >= 7U) return;
  for (y = 0U; y < 14U; y++)
  {
    for (x = 0U; x < 10U; x++)
    {
      /* 原字模为列数据，放大为 2x2 像素。 */
      Oled_SetPixel((uint8_t)(base_x + x),
                    (uint8_t)(base_y + y),
                    (uint8_t)((glyph[x / 2U] >> (y / 2U)) & 1U));
    }
  }
  oled_column++;
}

void Oled_WriteString(const char *text)
{
  while (text != NULL && *text != '\0') Oled_WriteChar(*text++);
}

HAL_StatusTypeDef Oled_Update(void)
{
  uint8_t page;
  uint8_t packet[OLED_WIDTH + 1U];
  if (oled_ready == 0U) return HAL_ERROR;
  packet[0] = 0x40U;
  for (page = 0U; page < 8U; page++)
  {
    if (Oled_Command((uint8_t)(0xB0U + page)) != HAL_OK) return HAL_ERROR;
    if (Oled_Command((uint8_t)(0x00U + (OLED_COLUMN_OFFSET & 0x0FU))) != HAL_OK ||
        Oled_Command((uint8_t)(0x10U + ((OLED_COLUMN_OFFSET >> 4) & 0x0FU))) != HAL_OK) return HAL_ERROR;
    memcpy(&packet[1], &oled_buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, packet,
                                sizeof(packet), 100U) != HAL_OK) return HAL_ERROR;
  }
  return HAL_OK;
}

uint8_t Oled_IsReady(void) { return oled_ready; }
