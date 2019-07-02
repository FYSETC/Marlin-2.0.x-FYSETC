/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Based on u8g_dev_st7920_128x64.c
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2011, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list
 *    of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "../../inc/MarlinConfigPre.h"

// #if HAS_GRAPHICAL_LCD // geo-f
#if ENABLED(U8GLIB_ST7920) && !defined(U8G_HAL_LINKS) && !defined(__SAM3X8E__)


#include <U8glib.h>

#include "HAL_LCD_com_defines.h"
#include "u8g_dev_st7920_128x64_stm32f103_HAL.h"

#define LCD_PIXEL_WIDTH  128
#define LCD_PIXEL_HEIGHT  64
#define PAGE_HEIGHT        8

/*
// init sequence from https://github.com/adafruit/ST7565-LCD/blob/master/ST7565/ST7565.cpp 
static const uint8_t u8g_dev_st7920_128x64_stm32f103_HAL_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),      // disable chip
  U8G_ESC_ADR(0),     // instruction mode
  U8G_ESC_RST(15),    // do reset low pulse with (15*16)+2 milliseconds (=maximum delay)
  U8G_ESC_DLY(100),   // 8 Dez 2012: additional delay 100 ms because of reset
  U8G_ESC_CS(1),      // enable chip
  U8G_ESC_DLY(50),    // delay 50 ms

  0x038,              // 8 Bit interface (DL=1), basic instruction set (RE=0)
  0x00C,              // display on, cursor & blink off; 0x08: all off
  0x006,              // Entry mode: Cursor move to right ,DDRAM address counter (AC) plus 1, no shift
  0x002,              // disable scroll, enable CGRAM adress
  0x001,              // clear RAM, needs 1.6 ms
  U8G_ESC_DLY(100),   // delay 100 ms

  U8G_ESC_CS(0),      // disable chip
  U8G_ESC_END         // end of sequence
};

void clear_graphics_DRAM_stm32f103(u8g_t *u8g, u8g_dev_t *dev) {
  ST7920_CS_ST();
  u8g_Delay(1);
  ST7920_SET_CMD_ST();
  ST7920_WRITE_BYTE_ST(0x08);       //display off, cursor+blink off
  ST7920_WRITE_BYTE_ST(0x3E);       //extended mode + GDRAM active
  for (uint8_t y = 0; y < (LCD_PIXEL_HEIGHT) / 2; y++) { //clear GDRAM
    ST7920_WRITE_BYTE_ST(0x80 | y); //set y
    ST7920_WRITE_BYTE_ST(0x80);     //set x = 0
    ST7920_SET_DAT_ST();
    for (uint8_t i = 0; i < 2 * (LCD_PIXEL_WIDTH) / 8; i++) //2x width clears both segments
      ST7920_WRITE_BYTE_ST(0);
    ST7920_SET_CMD_ST();
  }

  ST7920_WRITE_BYTE_ST(0x0C); //display on, cursor+blink off

  ST7920_NCS_ST();
}

uint8_t u8g_dev_st7920_128x64_stm32f103_HAL_4x_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  uint8_t y, i;
  switch (msg) {
    case U8G_DEV_MSG_INIT:       
      //u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      //u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_stm32f103_HAL_init_seq);
      //clear_graphics_DRAM_stm32f103(u8g, dev);      
            
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);      

      ST7920_CS_ST();
      u8g_Delay(120);                 // Initial delay for boot up
      ST7920_SET_CMD_ST();
      ST7920_WRITE_BYTE_ST(0x20);        // Non-extended mode
      ST7920_WRITE_BYTE_ST(0x08);        // Display off, cursor+blink off
      ST7920_WRITE_BYTE_ST(0x01);        // Clear DDRAM ram
      u8g_Delay(15);                  // Delay for DDRAM clear
      ST7920_WRITE_BYTE_ST(0x24);        // Extended mode
      ST7920_WRITE_BYTE_ST(0x26);        // Extended mode + GDRAM active
      for (y = 0; y < (LCD_PIXEL_HEIGHT) / 2; y++) {  // Clear GDRAM
        ST7920_WRITE_BYTE_ST(0x80 | y);  // Set y
        ST7920_WRITE_BYTE_ST(0x80);      // Set x = 0
        ST7920_SET_DAT_ST();        
        for (i = 0; i < 2 * (LCD_PIXEL_WIDTH) / 8; i++) // 2x width clears both segments
          ST7920_WRITE_BYTE_ST(0);
        ST7920_SET_CMD_ST();        
      }
      ST7920_WRITE_BYTE_ST(0x0C);        // Display on, cursor+blink off
      ST7920_NCS_ST();      
      break;
      
    case U8G_DEV_MSG_STOP:
      break;
      
    case U8G_DEV_MSG_PAGE_NEXT: {
   
      uint8_t *ptr;
      u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
      
      ST7920_SET_CMD_ST();
      ST7920_CS_ST();
      y = pb->p.page_y0;
      ptr = (uint8_t *)pb->buf;
      for (i = 0; i < 8; i ++) {
        ST7920_SET_CMD_ST();
        ST7920_WRITE_BYTE_ST(0x03E );      // enable extended mode 

        if (y < 32) {
          ST7920_WRITE_BYTE_ST(0x080 | y );      // y pos  
          ST7920_WRITE_BYTE_ST(0x080  );      // set x pos to 0
        }
        else {
          ST7920_WRITE_BYTE_ST(0x080 | (y-32) );      // y pos  
          ST7920_WRITE_BYTE_ST(0x080 | 8);      // set x pos to 64
        }
        
        ST7920_SET_DAT_ST();
        ST7920_WRITE_BYTES_ST(ptr, (LCD_PIXEL_WIDTH) / 8);
        ptr += (LCD_PIXEL_WIDTH) / 8;
        y++;
      }
      ST7920_NCS_ST();      
    }
    break;
  }
  return u8g_dev_pb8h1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_st7920_128x64_stm32f103_HAL_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  uint8_t i, y;
  switch (msg) {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_stm32f103_HAL_init_seq);
      clear_graphics_DRAM_stm32f103(u8g, dev);
      break;

    case U8G_DEV_MSG_STOP:
      break;

    case U8G_DEV_MSG_PAGE_NEXT: {     
      uint8_t *ptr;
      u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
      
      ST7920_SET_CMD_ST();
      ST7920_CS_ST();
      y = pb->p.page_y0;
      ptr = (uint8_t *)pb->buf;
      for (i = 0; i < 32; i ++) {        
        ST7920_SET_CMD_ST();
        ST7920_WRITE_BYTE_ST(0x03E );      // enable extended mode 

        if (y < 32) {
          ST7920_WRITE_BYTE_ST(0x080 | y );      // y pos  
          ST7920_WRITE_BYTE_ST(0x080  );      // set x pos to 0
        }
        else {
          ST7920_WRITE_BYTE_ST(0x080 | (y-32) );      // y pos  
          ST7920_WRITE_BYTE_ST(0x080 | 8);      // set x pos to 64
        }
        
        ST7920_SET_DAT_ST();
        ST7920_WRITE_BYTES_ST(ptr, (LCD_PIXEL_WIDTH) / 8);
        ptr += (LCD_PIXEL_WIDTH) / 8;
        y++;
      }
      ST7920_NCS_ST();
    }
    break;
  }
  return u8g_dev_pb32h1_base_fn(u8g, dev, msg, arg);
}


U8G_PB_DEV(u8g_dev_st7920_128x64_stm32f103_HAL_sw_spi, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_stm32f103_HAL_fn, U8G_COM_ST7920_HAL_SW_SPI);

#define QWIDTH ((LCD_PIXEL_WIDTH) * 4)
uint8_t u8g_dev_st7920_128x64_stm32f103_HAL_4x_buf[QWIDTH] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_st7920_128x64_stm32f103_HAL_4x_pb = { { 32, LCD_PIXEL_HEIGHT, 0, 0, 0 }, LCD_PIXEL_WIDTH, u8g_dev_st7920_128x64_stm32f103_HAL_4x_buf};
u8g_dev_t u8g_dev_st7920_128x64_stm32f103_HAL_4x_sw_spi = { u8g_dev_st7920_128x64_stm32f103_HAL_4x_fn, &u8g_dev_st7920_128x64_stm32f103_HAL_4x_pb, U8G_COM_ST7920_HAL_SW_SPI };

U8G_PB_DEV(u8g_dev_st7920_128x64_stm32f103_HAL_hw_spi, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_stm32f103_HAL_fn, u8g_com_stm32duino_hw_spi_fn);//U8G_COM_ST7920_HAL_HW_SPI
u8g_dev_t u8g_dev_st7920_128x64_stm32f103_HAL_4x_hw_spi = { u8g_dev_st7920_128x64_stm32f103_HAL_4x_fn, &u8g_dev_st7920_128x64_stm32f103_HAL_4x_pb, u8g_com_stm32duino_hw_spi_fn };//U8G_COM_ST7920_HAL_HW_SPI

#if defined(U8G_HAL_LINKS) || defined(__SAM3X8E__)
  // Also use this device for HAL version of rrd class. This results in the same device being used
  // for the ST7920 for HAL systems no matter what is selected in ultralcd_impl_DOGM.h.
  u8g_dev_t u8g_dev_st7920_128x64_rrd_sw_spi = { u8g_dev_st7920_128x64_HAL_4x_fn, &u8g_dev_st7920_128x64_HAL_4x_pb, U8G_COM_ST7920_HAL_SW_SPI };
#endif
*/

// NOTE:
// 20190615 - status screen is fine , but menu list name dots chaos , and the 16G sdcard can't read 
// need to edit U8glib-HAL_ID1932 u8g_SetPinLevel and U8G_COM_MSG_ADDRESS code saved in U8glib-HAL_ID1932_STM32_ST7920.zip

// Optimize this code with -O3
//#pragma GCC optimize (3)

uint8_t u8g_dev_st7920_128x64_stm32f103_HAL_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  uint8_t i, y;
  switch (msg) {
    case U8G_DEV_MSG_INIT: {
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      //OUT_WRITE(ST7920_CS_PIN, LOW);
      //OUT_WRITE(ST7920_DAT_PIN, LOW);
      //OUT_WRITE(ST7920_CLK_PIN, HIGH);

      ST7920_CS_ST();
      u8g_Delay(120);                 // Initial delay for boot up
      ST7920_SET_CMD_ST();
      ST7920_WRITE_BYTE_ST(0x20);        // Non-extended mode
      ST7920_WRITE_BYTE_ST(0x08);        // Display off, cursor+blink off
      ST7920_WRITE_BYTE_ST(0x01);        // Clear DDRAM ram
      u8g_Delay(15);                  // Delay for DDRAM clear
      ST7920_WRITE_BYTE_ST(0x24);        // Extended mode
      ST7920_WRITE_BYTE_ST(0x26);        // Extended mode + GDRAM active
      for (y = 0; y < (LCD_PIXEL_HEIGHT) / 2; y++) {  // Clear GDRAM
        ST7920_WRITE_BYTE_ST(0x80 | y);  // Set y
        ST7920_WRITE_BYTE_ST(0x80);      // Set x = 0
        ST7920_SET_DAT_ST();
        for (i = 0; i < 2 * (LCD_PIXEL_WIDTH) / 8; i++) // 2x width clears both segments
          ST7920_WRITE_BYTE_ST(0);
        ST7920_SET_CMD_ST();
      }
      ST7920_WRITE_BYTE_ST(0x0C);        // Display on, cursor+blink off
      ST7920_NCS_ST();
    }
    break;

    case U8G_DEV_MSG_STOP: break;

    case U8G_DEV_MSG_PAGE_NEXT: {
      uint8_t* ptr;
      u8g_pb_t* pb = (u8g_pb_t*)(dev->dev_mem);
      y = pb->p.page_y0;
      ptr = (uint8_t*)pb->buf;

      ST7920_CS_ST();
      for (i = 0; i < PAGE_HEIGHT; i ++) {
        ST7920_SET_CMD_ST();
        if (y < 32) { 
          ST7920_WRITE_BYTE_ST(0x80 | y);        // y
          ST7920_WRITE_BYTE_ST(0x80);            // x = 0
        }
        else {
          ST7920_WRITE_BYTE_ST(0x80 | (y - 32)); // y
          ST7920_WRITE_BYTE_ST(0x80 | 8);        // x = 64
        }
        ST7920_SET_DAT_ST();
        ST7920_WRITE_BYTES_ST(ptr, (LCD_PIXEL_WIDTH) / 8); // ptr incremented inside of macro!
        y++;
      }
      ST7920_NCS_ST();
    }
    break;
  }
  #if PAGE_HEIGHT == 8
    return u8g_dev_pb8h1_base_fn(u8g, dev, msg, arg);
  #elif PAGE_HEIGHT == 16
    return u8g_dev_pb16h1_base_fn(u8g, dev, msg, arg);
  #else
    return u8g_dev_pb32h1_base_fn(u8g, dev, msg, arg);
  #endif
}

uint8_t   u8g_dev_st7920_128x64_stm32f103_HAL_buf[(LCD_PIXEL_WIDTH) * (PAGE_HEIGHT) / 8] U8G_NOCOMMON;
u8g_pb_t  u8g_dev_st7920_128x64_stm32f103_HAL_pb = {{PAGE_HEIGHT, LCD_PIXEL_HEIGHT, 0, 0, 0}, LCD_PIXEL_WIDTH, u8g_dev_st7920_128x64_stm32f103_HAL_buf};
u8g_dev_t u8g_dev_st7920_128x64_stm32f103_HAL_hw_spi = {u8g_dev_st7920_128x64_stm32f103_HAL_fn, &u8g_dev_st7920_128x64_stm32f103_HAL_pb, &u8g_com_stm32duino_hw_spi_fn};

U8G_PB_DEV(u8g_dev_st7920_128x64_stm32f103_HAL_sw_spi, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_stm32f103_HAL_fn, U8G_COM_ST7920_HAL_SW_SPI);

#pragma GCC reset_options

#if ENABLED(LIGHTWEIGHT_UI)
  #include "../../HAL/shared/HAL_ST7920.h"
  void ST7920_cs()                          { ST7920_CS_ST(); }
  void ST7920_ncs()                         { ST7920_NCS_ST(); }
  void ST7920_set_cmd()                     { ST7920_SET_CMD_ST(); }
  void ST7920_set_dat()                     { ST7920_SET_DAT_ST(); }
  void ST7920_write_byte(const uint8_t val) { ST7920_WRITE_BYTE_ST(val); }
#endif

#endif // HAS_GRAPHICAL_LCD
