/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Juho Vähä-Herttua (juhovh@iki.fi)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "esp_i2c.h"

#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "user_interface.h"

static uint8 __gpiomux[16] = {
  PERIPHS_IO_MUX_GPIO0_U,
  PERIPHS_IO_MUX_U0TXD_U,
  PERIPHS_IO_MUX_GPIO2_U,
  PERIPHS_IO_MUX_U0RXD_U,
  PERIPHS_IO_MUX_GPIO4_U,
  PERIPHS_IO_MUX_GPIO5_U,
  PERIPHS_IO_MUX_SD_CLK_U,
  PERIPHS_IO_MUX_SD_DATA0_U,
  PERIPHS_IO_MUX_SD_DATA1_U,
  PERIPHS_IO_MUX_SD_DATA2_U,
  PERIPHS_IO_MUX_SD_DATA3_U,
  PERIPHS_IO_MUX_SD_CMD_U,
  PERIPHS_IO_MUX_MTDI_U,
  PERIPHS_IO_MUX_MTCK_U,
  PERIPHS_IO_MUX_MTMS_U,
  PERIPHS_IO_MUX_MTDO_U
};
static uint8 __gpiofunc[16] = {
  FUNC_GPIO0,
  FUNC_GPIO1,
  FUNC_GPIO2,
  FUNC_GPIO3,
  FUNC_GPIO4,
  FUNC_GPIO5,
  FUNC_SDCLK,
  FUNC_SDDATA0,
  FUNC_SDDATA1,
  FUNC_GPIO9,
  FUNC_GPIO10,
  FUNC_SDCMD,
  FUNC_GPIO12,
  FUNC_GPIO13,
  FUNC_GPIO14,
  FUNC_GPIO15
};

#define PERIPHS_IO_MUX_GPIO(pin_no)  (PERIPHS_IO_MUX + __gpiomux[pin_no & 0x0f])
#define FUNC_GPIO(pin_no)            (__gpiofunc[pin_no & 0x0f])

#define HIGH  1
#define LOW   0

#define SDA_SET(bit)  GPIO_OUTPUT_SET(i2c_sda, !!bit)
#define SDA_GET()     GPIO_INPUT_GET(i2c_sda)
#define SCL_SET(bit)  GPIO_OUTPUT_SET(i2c_scl, !!bit)
#define SCL_GET()     GPIO_INPUT_GET(i2c_scl)

static uint8 i2c_delay_us;
static uint8 i2c_sda, i2c_scl;
static uint32 i2c_cslimit;

#define I2C_DELAY()   os_delay_us(i2c_delay_us)
#define I2C_CLOCK_STRETCH() do {                \
  uint32 i=0;                             \
  while (SCL_GET() == 0 && i++ < i2c_cslimit);  \
} while(0)



static void ICACHE_FLASH_ATTR
i2c_set_gpio_pad_driver(uint8 pin_no, uint8 val)
{
  uint32 pinaddr = GPIO_PIN_ADDR(GPIO_ID_PIN(pin_no));
  uint32 status = GPIO_REG_READ(pinaddr) & (~GPIO_PIN_PAD_DRIVER_MASK);

  GPIO_REG_WRITE(pinaddr, status | GPIO_PIN_PAD_DRIVER_SET(val));
}

static uint8 ICACHE_FLASH_ATTR
i2c_write_start()
{
  SCL_SET(HIGH);
  SDA_SET(HIGH);
  if (SDA_GET() == LOW) {
    return 1;
  }
  I2C_DELAY();
  SDA_SET(LOW);
  I2C_DELAY();

  return 0;
}

static void ICACHE_FLASH_ATTR
i2c_write_stop()
{
  SCL_SET(LOW);
  SDA_SET(LOW);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  I2C_DELAY();
  SDA_SET(HIGH);
  I2C_DELAY();
}

static void ICACHE_FLASH_ATTR
i2c_write_bit(bool bit)
{
  SCL_SET(LOW);
  SDA_SET(bit);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  I2C_DELAY();
}

static uint8 ICACHE_FLASH_ATTR
i2c_read_bit()
{
  uint8 bit;

  SCL_SET(LOW);
  SDA_SET(HIGH);
  I2C_DELAY();
  SCL_SET(HIGH);
  I2C_CLOCK_STRETCH();
  bit = SDA_GET();
  I2C_DELAY();

  return bit;
}

static uint8 ICACHE_FLASH_ATTR
i2c_write_byte(uint8 byte)
{
  uint8 bit;

  for (bit = 0; bit < 8; bit++) {
    i2c_write_bit(byte & 0x80);
    byte <<= 1;
  }

  // Read either ACK or NACK
  return i2c_read_bit();
}

static uint8 ICACHE_FLASH_ATTR
i2c_read_byte(uint8 nack)
{
  uint8 byte = 0;
  uint8 bit;

  for (bit = 0; bit < 8; bit++) {
    byte = (byte << 1);
    byte |= i2c_read_bit();
  }
  i2c_write_bit(nack);

  return byte;
}



void ICACHE_FLASH_ATTR
esp_i2c_init(uint8 sda, uint8 scl)
{
  ETS_GPIO_INTR_DISABLE();

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO(sda), FUNC_GPIO(sda));
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO(scl), FUNC_GPIO(scl));

  i2c_set_gpio_pad_driver(sda, GPIO_PAD_DRIVER_ENABLE);
  i2c_set_gpio_pad_driver(scl, GPIO_PAD_DRIVER_ENABLE);

  GPIO_DIS_OUTPUT(sda);
  GPIO_DIS_OUTPUT(scl);

  ETS_GPIO_INTR_ENABLE();

  i2c_sda = sda;
  i2c_scl = scl;

  esp_i2c_set_clock(100000);
  esp_i2c_set_clock_stretch_limit(800);
}

void ICACHE_FLASH_ATTR
esp_i2c_stop()
{
  ETS_GPIO_INTR_DISABLE();

  i2c_set_gpio_pad_driver(i2c_sda, GPIO_PAD_DRIVER_DISABLE);
  i2c_set_gpio_pad_driver(i2c_scl, GPIO_PAD_DRIVER_DISABLE);

  GPIO_DIS_OUTPUT(i2c_sda);
  GPIO_DIS_OUTPUT(i2c_scl);

  ETS_GPIO_INTR_ENABLE();
}

void ICACHE_FLASH_ATTR
esp_i2c_set_clock(uint32 freq)
{
  // Divide one second (in us) with frequency
  i2c_delay_us = 1000 / (freq / 1000);
}

void ICACHE_FLASH_ATTR
esp_i2c_set_clock_stretch_limit(uint32 limit)
{
  if (system_get_cpu_freq() == SYS_CPU_160MHZ) {
    i2c_cslimit = 2 * limit;
  } else {
    i2c_cslimit = limit;
  }
}


uint8 ICACHE_FLASH_ATTR
esp_i2c_read_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop)
{
  uint32 i;

  if (i2c_write_start()) {
    // Received SDA low when writing start
    return 1;
  }
  if (i2c_write_byte(((address << 1) | 1) & 0xFF)) {
    // Received NACK when writing address
    if (send_stop) i2c_write_stop();
    return 2;
  }

  for (i=0; i < len-1; i++) {
    buf[i] = i2c_read_byte(0);
  }
  buf[i] = i2c_read_byte(1);
  if (send_stop) i2c_write_stop();

  return 0;
}

uint8 ICACHE_FLASH_ATTR
esp_i2c_write_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop)
{
  uint32 i;

  if (i2c_write_start()) {
    // Received SDA low when writing start
    return 1;
  }
  if (i2c_write_byte((address << 1) & 0xFF)) {
    // Received NACK when writing address
    if (send_stop) i2c_write_stop();
    return 2;
  }

  for (i=0; i < len; i++) {
    if (i2c_write_byte(buf[i])) {
      // Received NACK when writing data
      if (send_stop) i2c_write_stop();
      return 3;
    }
  }
  if (send_stop) i2c_write_stop();

  return 0;
}
