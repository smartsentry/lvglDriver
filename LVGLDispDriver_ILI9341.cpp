
/* lvglDriver for Mbed
 * Copyright (c) 2019 Johannes Stratmann
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "LVGLDispDriver_ILI9341.h"




// static const uint8_t init_cmd_list[] = {
//     ILI9341_SWRESET,    0,                  //Soft Reset
//     TFTLCD_DELAY8,      5,                // .kbv will power up with ONLY reset, sleep out, display on
//     ILI9341_DISPOFF,    0,                  //Display Off
//     ILI9341_PWCTRLB,    3, 0x00, 0x83, 0x30,
//     ILI9341_PWSEQCTRL,  4, 0x64, 0x03, 0x12, 0x81,
//     ILI9341_TIMECTRLA_INT, 3, 0x85, 0x01, 0x79,
//     ILI9341_PWCTRLA,     5,  0x39, 0x2C, 0x00, 0x34, 0x02,
//     ILI9341_PUMPRATIO,  1, 0x20,
//     ILI9341_TIMECTRLB,     2,  0x00, 0x00,
//     ILI9341_PWCTRL1,     1,  0x26,
//     ILI9341_PWCTRL2,     1,  0x11,
//     ILI9341_VMCTRL1,    2, 0x35, 0x3E,
//     ILI9341_VMCTRL2,    1, 0xBE,
//     ILI9341_MADCTL,     1, 0x68,//disp rotation
//     ILI9341_PIXSET,     1, 0x55,            //Pixel read=565, write=565.
//     ILI9341_FRMCTR1,    2, 0x00, 0x1B,
//     ILI9341_GAM3CTRL,   1, 0x08,
//     ILI9341_GAMSET,     1, 0x01,
//     ILI9341_PGAMCTRL,   14, 0xF0, 0x03, 0x07, 0x0B, 0x09, 0x06, 0x21, 0x54, 0x31, 0x36, 0x0E, 0x10, 0x10, 0x1A,
//     ILI9341_NGAMCTRL,   14, 0xF0, 0x01, 0x04, 0x07, 0x06, 0x04, 0x20, 0x44, 0x31, 0x33, 0x06, 0x07, 0x09, 0x11,
//     //Tearing?
//     ILI9341_ETMOD,      1, 0x07,
//     ILI9341_DISCTRL,    4, 0x0A, 0x82, 0x27, 0x00,
//     ILI9341_SLPOUT,     0,
//     TFTLCD_DELAY8,      100,
//     ILI9341_DISPON,     0,
//     TFTLCD_DELAY8,      100

// };
    // ILI9341_PGAMCTRL,   15, 0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0x87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00,
    // ILI9341_NGAMCTRL,   15, 0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F,
    // ILI9341_PGAMCTRL,   14, 0xF0, 0x03, 0x07, 0x0B, 0x09, 0x06, 0x21, 0x54, 0x31, 0x36, 0x0E, 0x10, 0x10, 0x1A,
    // ILI9341_NGAMCTRL,   14, 0xF0, 0x01, 0x04, 0x07, 0x06, 0x04, 0x20, 0x44, 0x31, 0x33, 0x06, 0x07, 0x09, 0x11,
LVGLDispILI9341::LVGLDispILI9341(SPI &spi, PinName pinCS, PinName pinCMD, PinName pinRST, PinName pinBacklight, 
                               uint32_t nBufferRows, uint32_t resolutionX, uint32_t resolutionY) :
    LVGLDispDriver(resolutionX, resolutionY),
    _spi(spi),
    _cs(pinCS),
    _cmd(pinCMD),
    _rst(pinRST),
    _backlight(pinBacklight),
    _nBufferRows(nBufferRows)
{
    // tft controller init
    tft_init();

    // lvgl init
    init();
}

void LVGLDispILI9341::init()
{

 size_t bufferSize = _horRes * _nBufferRows;

    // allocate memory for display buffer 1
    _buf1_1 = new uint16_t[bufferSize];             /* a buffer for n rows */
    MBED_ASSERT(_buf1_1 != nullptr);

    // allocate memory for display buffer 2
    _buf2_1 = new uint16_t[bufferSize];             /* a buffer for n rows */
    MBED_ASSERT(_buf2_1 != nullptr);

    _disp = lv_display_create(_horRes, _verRes);
    lv_display_set_flush_cb(_disp, disp_flush);
    lv_display_set_buffers(_disp, _buf1_1, _buf2_1, bufferSize*2, LV_DISP_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(_disp, this);

}

void LVGLDispILI9341::disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    LVGLDispILI9341* instance = (LVGLDispILI9341*)lv_display_get_user_data(disp);
    instance->flush(area, px_map);

}

void LVGLDispILI9341::flush(const lv_area_t *area, uint8_t * px_map)
{
  	//_spi.format(8, 0);		// switch to 8 bit transfer for commands

    set_addr_win(area->x1, area->y1, area->x2, area->y2);  	// set display area

    int pixels = lv_area_get_size(area); 	// in bytes

//As 16 bit transfer
  	// _spi.format(16, 0);		// switch to 16 bit transfer for data
    // [[maybe_unused]] volatile int rc = _spi.transfer((uint16_t *)px_map, pixels*2 , nullptr,  0, callback(this, &LVGLDispILI9341::flush_ready));

//As 16 bit DMA transfer
    _spi.set_dma_usage(DMA_USAGE_ALWAYS);
  	_spi.format(16, 0);		// switch to 16 bit transfer for data
    [[maybe_unused]] volatile int rc = _spi.transfer((uint16_t *)px_map, pixels*2 , nullptr,  0, callback(this, &LVGLDispILI9341::flush_ready));

//As 8 bit DMA transfer
    // _spi.set_dma_usage(DMA_USAGE_ALWAYS);
    // lv_draw_sw_rgb565_swap(px_map,pixels);
    // [[maybe_unused]] volatile int rc = _spi.transfer(px_map, pixels*2 , nullptr,  0, callback(this, &LVGLDispILI9341::flush_ready));


}

void LVGLDispILI9341::flush_ready(int event_flags)
{
    if (event_flags & SPI_EVENT_COMPLETE) {
        _cs = 1;
        lv_disp_flush_ready(_disp);         /* Indicate you are ready with the flushing*/
        //look at lv_display_set_flush_wait_cb?`
    }
}

/**
 * Write a command to the GC9A01
 * @param cmd the command
 */
void LVGLDispILI9341::command(uint8_t cmd)
{
    _spi.format(8, 0);		// switch back to 8 bit transfer
    _cs = 0;
	_cmd = 0;
    _spi.write(cmd);
    _cmd = 1;
}

/**
 * Write data to the GC9A01
 * @param data the data
 */
void LVGLDispILI9341::data(uint8_t data)
{
	
    _spi.write(data);
}


// hard reset of the tft controller

void LVGLDispILI9341::hard_reset()
{
    if (_rst.is_connected()) {
        _rst = 1;
        ThisThread::sleep_for(50ms);
        _rst = 0;
        ThisThread::sleep_for(50ms);
        _rst = 1;
        ThisThread::sleep_for(50ms);
    }
}

void LVGLDispILI9341::write_table(const uint8_t *table, int16_t size)
{
    while (size > 0) {
        uint8_t cmd = *table++;
        uint8_t len = *table++;
        if (cmd == TFTLCD_DELAY8) {
            ThisThread::sleep_for(1ms *len);
            len = 0;
        } else {
            int n = len;

            command(cmd);
            while (n-- > 0) {
                data(*table++);
            }
            _cs = 1;
        }
        size -= len + 2;
    }
}

void LVGLDispILI9341::setRotation(uint8_t m) 
{
    uint16_t t;

    switch (m) {
        case 1:
            t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
            break;
        case 2:
            t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
            break;
        case 3:
            t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
            break;
        case 0:
        default:
            t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
            break;
    }
    command(ILI9341_MADCTL);
    data(t); 
}



void LVGLDispILI9341::set_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    command(ILI9341_CASET); // Column addr set
    data(x0 >> 8);
    data(x0);     // XSTART 
    data(x1 >> 8);
    data(x1);     // XEND
    _cs = 1;
    
    command(ILI9341_PASET); // Row addr set
    data(y0 >> 8);
    data(y0);     // YSTART
    data(y1 >> 8);
    data(y1);     // YEND
    _cs = 1;

    command(ILI9341_RAMWR);
}

int LVGLDispILI9341::tft_init()
{
	hard_reset();
    write_table(init_cmd_list, sizeof(init_cmd_list));
    //setRotation(2);

    _backlight =1;
	
	return 0;
}