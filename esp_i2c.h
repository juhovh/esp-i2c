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
#ifndef ESP_I2C_H
#define ESP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <c_types.h>

void esp_i2c_init(uint8 sda, uint8 scl);
void esp_i2c_stop();
void esp_i2c_set_clock(uint32 freq);
void esp_i2c_set_clock_stretch_limit(uint32 limit);
uint8 esp_i2c_write_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop);
uint8 esp_i2c_read_buf(uint8 address, uint8 *buf, uint32 len, uint8 send_stop);

#ifdef __cplusplus
}
#endif

#endif
