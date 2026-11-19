
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

/*
 * Register definitions and the controller init table (init_cmd_list) are in
 * LVGLDispDriver_ILI9341.h so that panel-specific subclasses such as
 * KD024QVRMA041 can use them.
 *
 * Chip select is handled by the SPI object (constructed with ssel):
 * _spi.select() / _spi.deselect().
 */

LVGLDispILI9341::LVGLDispILI9341(SPI &spi, PinName pinCMD, PinName pinRST, PinName pinBacklight, 
                               uint32_t nBufferRows, uint32_t resolutionX, uint32_t resolutionY) :
    LVGLDispDriver(resolutionX, resolutionY),
    _spi(spi),
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
    size_t bufferSize = _horRes * _nBufferRows;     // in pixels

    // allocate memory for display buffer
    _buf1_1 = new uint16_t[bufferSize];             /* a buffer for n rows */
    MBED_ASSERT(_buf1_1 != nullptr);
    _buf2_1 = nullptr;                              // single buffer

    _disp = lv_display_create(_horRes, _verRes);
    lv_display_set_flush_cb(_disp, disp_flush);
    lv_display_set_buffers(_disp, _buf1_1, nullptr, bufferSize * sizeof(uint16_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(_disp, this);
}

void LVGLDispILI9341::disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    LVGLDispILI9341* instance = (LVGLDispILI9341*)lv_display_get_user_data(disp);

    instance->flush(area, px_map);

    // lv_display_flush_ready() is called by the async SPI transfer
}

void LVGLDispILI9341::flush(const lv_area_t *area, uint8_t *px_map)
{
    set_addr_win(area->x1, area->y1, area->x2, area->y2);  	// set display area, leaves CS low

    int len = lv_area_get_size(area) * 2; 	// in bytes

    // 16 bit DMA transfer of the pixel data
    _spi.set_dma_usage(DMA_USAGE_ALWAYS);
  	_spi.format(16, 0);		// switch to 16 bit transfer for data
    [[maybe_unused]] volatile int rc = _spi.transfer((uint16_t*)px_map, len, nullptr, 0, callback(this, &LVGLDispILI9341::flush_ready));
    _spi.deselect();        // the transfer holds the SPI lock; deselect is not allowed in the ISR callback
}

void LVGLDispILI9341::flush_ready(int event_flags)
{
    if (event_flags & SPI_EVENT_COMPLETE) {
        lv_display_flush_ready(_disp);         /* Indicate you are ready with the flushing*/
    }
}

/**
 * Write a command to the ILI9341, leaves CS low for following data
 * @param cmd the command
 */
void LVGLDispILI9341::command(uint8_t cmd)
{
    _spi.format(8, 0);		// switch to 8 bit transfer for commands
    _spi.select();
	_cmd = 0;
    _spi.write(cmd);
    _cmd = 1;
}

/**
 * Write data to the ILI9341
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
            ThisThread::sleep_for(1ms * len);
            len = 0;
        } else {
            int n = len;

            command(cmd);
            while (n-- > 0) {
                data(*table++);
            }
            _spi.deselect();
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
    _spi.deselect();
}

int LVGLDispILI9341::tft_init()
{
	hard_reset();
    write_table(init_cmd_list, sizeof(init_cmd_list));      // includes rotation (MADCTL)

    // backlight is switched on by the application with backlightEnable()
	
	return 0;
}

void LVGLDispILI9341::backlightEnable(bool enable)
{
    _backlight = enable;
}

void LVGLDispILI9341::set_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    command(ILI9341_CASET); // Column addr set
    data(x0 >> 8);
    data(x0);     // XSTART 
    data(x1 >> 8);
    data(x1);     // XEND
    _spi.deselect();
    
    command(ILI9341_PASET); // Row addr set
    data(y0 >> 8);
    data(y0);     // YSTART
    data(y1 >> 8);
    data(y1);     // YEND
    _spi.deselect();

    command(ILI9341_RAMWR);
}
