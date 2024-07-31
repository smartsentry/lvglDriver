
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

/*********************
 *      DEFINES
 *********************/
#define ILI9341_CMD_MODE    0
#define ILI9341_DATA_MODE   1

#define ILI9341_TFTWIDTH    240
#define ILI9341_TFTHEIGHT   320

/* Level 1 Commands -------------- [section] Description */

#define ILI9341_NOP         0x00 /* [8.2.1 ] No Operation / Terminate Frame Memory Write */
#define ILI9341_SWRESET     0x01 /* [8.2.2 ] Software Reset */
#define ILI9341_RDDIDIF     0x04 /* [8.2.3 ] Read Display Identification Information */
#define ILI9341_RDDST       0x09 /* [8.2.4 ] Read Display Status */
#define ILI9341_RDDPM       0x0A /* [8.2.5 ] Read Display Power Mode */
#define ILI9341_RDDMADCTL   0x0B /* [8.2.6 ] Read Display MADCTL */
#define ILI9341_RDDCOLMOD   0x0C /* [8.2.7 ] Read Display Pixel Format */
#define ILI9341_RDDIM       0x0D /* [8.2.8 ] Read Display Image Mode */
#define ILI9341_RDDSM       0x0E /* [8.2.9 ] Read Display Signal Mode */
#define ILI9341_RDDSDR      0x0F /* [8.2.10] Read Display Self-Diagnostic Result */
#define ILI9341_SLPIN       0x10 /* [8.2.11] Enter Sleep Mode */
#define ILI9341_SLPOUT      0x11 /* [8.2.12] Leave Sleep Mode */
#define ILI9341_PTLON       0x12 /* [8.2.13] Partial Display Mode ON */
#define ILI9341_NORON       0x13 /* [8.2.14] Normal Display Mode ON */
#define ILI9341_DINVOFF     0x20 /* [8.2.15] Display Inversion OFF */
#define ILI9341_DINVON      0x21 /* [8.2.16] Display Inversion ON */
#define ILI9341_GAMSET      0x26 /* [8.2.17] Gamma Set */
#define ILI9341_DISPOFF     0x28 /* [8.2.18] Display OFF*/
#define ILI9341_DISPON      0x29 /* [8.2.19] Display ON*/
#define ILI9341_CASET       0x2A /* [8.2.20] Column Address Set */
#define ILI9341_PASET       0x2B /* [8.2.21] Page Address Set */
#define ILI9341_RAMWR       0x2C /* [8.2.22] Memory Write */
#define ILI9341_RGBSET      0x2D /* [8.2.23] Color Set (LUT for 16-bit to 18-bit color depth conversion) */
#define ILI9341_RAMRD       0x2E /* [8.2.24] Memory Read */
#define ILI9341_PTLAR       0x30 /* [8.2.25] Partial Area */
#define ILI9341_VSCRDEF     0x33 /* [8.2.26] Veritcal Scrolling Definition */
#define ILI9341_TEOFF       0x34 /* [8.2.27] Tearing Effect Line OFF */
#define ILI9341_TEON        0x35 /* [8.2.28] Tearing Effect Line ON */
#define ILI9341_MADCTL      0x36 /* [8.2.29] Memory Access Control */
#define ILI9341_MADCTL_MY   0x80 /*          MY row address order */
#define ILI9341_MADCTL_MX   0x40 /*          MX column address order */
#define ILI9341_MADCTL_MV   0x20 /*          MV row / column exchange */
#define ILI9341_MADCTL_ML   0x10 /*          ML vertical refresh order */
#define ILI9341_MADCTL_MH   0x04 /*          MH horizontal refresh order */
#define ILI9341_MADCTL_RGB  0x00 /*          RGB Order [default] */
#define ILI9341_MADCTL_BGR  0x08 /*          BGR Order */
#define ILI9341_VSCRSADD    0x37 /* [8.2.30] Vertical Scrolling Start Address */
#define ILI9341_IDMOFF      0x38 /* [8.2.31] Idle Mode OFF */
#define ILI9341_IDMON       0x39 /* [8.2.32] Idle Mode ON */
#define ILI9341_PIXSET      0x3A /* [8.2.33] Pixel Format Set */
#define ILI9341_WRMEMCONT   0x3C /* [8.2.34] Write Memory Continue */
#define ILI9341_RDMEMCONT   0x3E /* [8.2.35] Read Memory Continue */
#define ILI9341_SETSCANTE   0x44 /* [8.2.36] Set Tear Scanline */
#define ILI9341_GETSCAN     0x45 /* [8.2.37] Get Scanline */
#define ILI9341_WRDISBV     0x51 /* [8.2.38] Write Display Brightness Value */
#define ILI9341_RDDISBV     0x52 /* [8.2.39] Read Display Brightness Value */
#define ILI9341_WRCTRLD     0x53 /* [8.2.40] Write Control Display */
#define ILI9341_RDCTRLD     0x54 /* [8.2.41] Read Control Display */
#define ILI9341_WRCABC      0x55 /* [8.2.42] Write Content Adaptive Brightness Control Value */
#define ILI9341_RDCABC      0x56 /* [8.2.43] Read Content Adaptive Brightness Control Value */
#define ILI9341_WRCABCMIN   0x5E /* [8.2.44] Write CABC Minimum Brightness */
#define ILI9341_RDCABCMIN   0x5F /* [8.2.45] Read CABC Minimum Brightness */
#define ILI9341_RDID1       0xDA /* [8.2.46] Read ID1 - Manufacturer ID (user) */
#define ILI9341_RDID2       0xDB /* [8.2.47] Read ID2 - Module/Driver version (supplier) */
#define ILI9341_RDID3       0xDC /* [8.2.48] Read ID3 - Module/Driver version (user) */

/* Level 2 Commands -------------- [section] Description */

#define ILI9341_IFMODE      0xB0 /* [8.3.1 ] Interface Mode Control */
#define ILI9341_FRMCTR1     0xB1 /* [8.3.2 ] Frame Rate Control (In Normal Mode/Full Colors) */
#define ILI9341_FRMCTR2     0xB2 /* [8.3.3 ] Frame Rate Control (In Idle Mode/8 colors) */
#define ILI9341_FRMCTR3     0xB3 /* [8.3.4 ] Frame Rate control (In Partial Mode/Full Colors) */
#define ILI9341_INVTR       0xB4 /* [8.3.5 ] Display Inversion Control */
#define ILI9341_PRCTR       0xB5 /* [8.3.6 ] Blanking Porch Control */
#define ILI9341_DISCTRL     0xB6 /* [8.3.7 ] Display Function Control */
#define ILI9341_ETMOD       0xB7 /* [8.3.8 ] Entry Mode Set */
#define ILI9341_BLCTRL1     0xB8 /* [8.3.9 ] Backlight Control 1 - Grayscale Histogram UI mode */
#define ILI9341_BLCTRL2     0xB9 /* [8.3.10] Backlight Control 2 - Grayscale Histogram still picture mode */
#define ILI9341_BLCTRL3     0xBA /* [8.3.11] Backlight Control 3 - Grayscale Thresholds UI mode */
#define ILI9341_BLCTRL4     0xBB /* [8.3.12] Backlight Control 4 - Grayscale Thresholds still picture mode */
#define ILI9341_BLCTRL5     0xBC /* [8.3.13] Backlight Control 5 - Brightness Transition time */
#define ILI9341_BLCTRL7     0xBE /* [8.3.14] Backlight Control 7 - PWM Frequency */
#define ILI9341_BLCTRL8     0xBF /* [8.3.15] Backlight Control 8 - ON/OFF + PWM Polarity*/
#define ILI9341_PWCTRL1     0xC0 /* [8.3.16] Power Control 1 - GVDD */
#define ILI9341_PWCTRL2     0xC1 /* [8.3.17] Power Control 2 - step-up factor for operating voltage */
#define ILI9341_VMCTRL1     0xC5 /* [8.3.18] VCOM Control 1 - Set VCOMH and VCOML */
#define ILI9341_VMCTRL2     0xC7 /* [8.3.19] VCOM Control 2 - VCOM offset voltage */
#define ILI9341_NVMWR       0xD0 /* [8.3.20] NV Memory Write */
#define ILI9341_NVMPKEY     0xD1 /* [8.3.21] NV Memory Protection Key */
#define ILI9341_RDNVM       0xD2 /* [8.3.22] NV Memory Status Read */
#define ILI9341_RDID4       0xD3 /* [8.3.23] Read ID4 - IC Device Code */
#define ILI9341_PGAMCTRL    0xE0 /* [8.3.24] Positive Gamma Control */
#define ILI9341_NGAMCTRL    0xE1 /* [8.3.25] Negative Gamma Correction */
#define ILI9341_DGAMCTRL1   0xE2 /* [8.3.26] Digital Gamma Control 1 */
#define ILI9341_DGAMCTRL2   0xE3 /* [8.3.27] Digital Gamma Control 2 */
#define ILI9341_IFCTL       0xF6 /* [8.3.28] 16bits Data Format Selection */

/* Extended Commands --------------- [section] Description*/

#define ILI9341_PWCTRLA       0xCB /* [8.4.1] Power control A */
#define ILI9341_PWCTRLB       0xCF /* [8.4.2] Power control B */
#define ILI9341_TIMECTRLA_INT 0xE8 /* [8.4.3] Internal Clock Driver timing control A */
#define ILI9341_TIMECTRLA_EXT 0xE9 /* [8.4.4] External Clock Driver timing control A */
#define ILI9341_TIMECTRLB     0xEA /* [8.4.5] Driver timing control B (gate driver timing control) */
#define ILI9341_PWSEQCTRL     0xED /* [8.4.6] Power on sequence control */
#define ILI9341_GAM3CTRL      0xF2 /* [8.4.7] Enable 3 gamma control */
#define ILI9341_PUMPRATIO     0xF7 /* [8.4.8] Pump ratio control */

#define TFTLCD_DELAY8 0xFF

static const uint8_t ILI9341_regValues_ada[] = {        
    // Adafruit_TFTLCD only works with EXTC=0
    //                     0xF6, 3, 0x00, 0x01, 0x00,  //Interface Control needs EXTC=1 TM=0, RIM=0
    //            0xF6, 3, 0x01, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=1, RIM=1
    /**/
    //0xF6, 3, 0x09, 0x01, 0x03,        //Interface Control needs EXTC=1 RM=0, RIM=1
    ILI9341_IFMODE,     1, 0x40,        //RGB Signal [40] RCM=2
    ILI9341_INVTR,      1, 0x00,        //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
    ILI9341_PWCTRL1,    1, 0x23,        //Power Control 1 [26]
    ILI9341_PWCTRL2,    1, 0x10,        //Power Control 2 [00]
    ILI9341_VMCTRL1,    2, 0x2B, 0x2B,  //VCOM 1 [31 3C]
    ILI9341_VMCTRL2,    1, 0xC0,        //VCOM 2 [C0]
    ILI9341_MADCTL,     1, 0x48,        //Memory Access [00]
    ILI9341_FRMCTR1,    2, 0x00, 0x1B,  //Frame Control [00 1B]
    ILI9341_ETMOD,      1, 0x07,        //Entry Mode [00]
    TFTLCD_DELAY8,      150
};

static const uint8_t ILI9341_regValues_post[] = {
    ILI9341_MADCTL,     1, 0x48,        //Memory Access [00]
    ILI9341_VSCRDEF,    6, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00,
    ILI9341_VSCRSADD,   2, 0x00, 0x00,
    ILI9341_NORON,      0,              // normaldisp
    ILI9341_DINVON,     0,              // invert off
};

static const uint8_t reset_off[] = {
    ILI9341_SWRESET,    0,                  //Soft Reset
    TFTLCD_DELAY8,      150,                // .kbv will power up with ONLY reset, sleep out, display on
    ILI9341_DISPOFF,    0,                  //Display Off
    ILI9341_PIXSET,     1, 0x55,            //Pixel read=565, write=565.
    // Power control
    // ILI9341_PWCTRL1,    2, 0x00, 0x26,
    // ILI9341_PWCTRL2,    2, 0x00, 0x11,
    // ILI9341_VMCTRL1,    4, 0x00, 0x5c, 0x00, 0x4c,
    // ILI9341_VMCTRL2,    2, 0x00, 0x94,
    // frame rate
    // ILI9341_FRMCTR1,    4, 0x00, 0x00, 0x00, 0x1b
};

static const uint8_t wake_on[] = {
    ILI9341_SLPOUT,     0,                  //Sleep Out
    TFTLCD_DELAY8,      150,
    ILI9341_DISPON,     0,                  //Display On
    //additional settings
    // ILI9341_DINVON,     0,                  // invert off
    ILI9341_MADCTL,     1, 0x48            //Memory Access
    //ILI9341_IFMODE,     1, 0x40             //RGB Signal [40] RCM=2
};

static lv_color_t framebuffer[240*16] __attribute__((section(".ram_ccm")));
// extern uint32_t __ram_ccm_start__;

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
    size_t bufferSize = _horRes * 16; //_nBufferRows;

    // allocate memory for display buffer
    // _buf1_1 = new lv_color_t[bufferSize];             /* a buffer for n rows */
    // MBED_ASSERT(_buf1_1 != nullptr);
    // memset(_buf1_1, 0, bufferSize*sizeof(lv_color_t));
    // _buf1_1 = (lv_color_t*)&__ram_ccm_start__;
    _buf1_1 = framebuffer;
    

    lv_disp_draw_buf_init(&_disp_buf_1, _buf1_1, NULL, bufferSize);   /* Initialize the display buffer */

    /*Finally register the driver*/
    _disp_drv.flush_cb = disp_flush;
    _disp_drv.draw_buf = &_disp_buf_1;
    _disp_drv.user_data = this;
    _disp = lv_disp_drv_register(&_disp_drv);
}

void LVGLDispILI9341::disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    LVGLDispILI9341* instance = (LVGLDispILI9341*)disp_drv->user_data;

    instance->flush(area, color_p);

    // lv_disp_flush_ready(disp_drv);                 // called by async SPI transfer
}

void LVGLDispILI9341::flush(const lv_area_t *area, lv_color_t *color_p)
{
  	_spi.format(8, 0);		// switch to 8 bit transfer for commands

    _cs = 0;
    set_addr_win(area->x1, area->y1, area->x2, area->y2);  	// set display area

  	_spi.format(16, 0);		// switch to 16 bit transfer for data
  	_cmd = 1;

    // int32_t len = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1); 	// in 16 bit words
    // _spi.write((const char*)color_p, len, nullptr, 0);						// transfer pixel data

    int len = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2; 	// in bytes
    [[maybe_unused]] volatile int rc = _spi.transfer((uint16_t*)color_p, len, nullptr,  0, callback(this, &LVGLDispILI9341::flush_ready));
}

void LVGLDispILI9341::flush_ready(int event_flags)
{
    if (event_flags & SPI_EVENT_COMPLETE) {
        _cs = 1;
        lv_disp_flush_ready(&_disp_drv);         /* Indicate you are ready with the flushing*/
    }
}

/**
 * Write a command to the GC9A01
 * @param cmd the command
 */
void LVGLDispILI9341::command(uint8_t cmd)
{
	_cmd = 0;
    _spi.write(cmd);
}

/**
 * Write data to the GC9A01
 * @param data the data
 */
void LVGLDispILI9341::data(uint8_t data)
{
	_cmd = 1;
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
            wait_us(len * 1000);  // ThisThread::sleep_for(Kernel::Clock::duration_u32 {len});
            len = 0;
        } else {
            int n = len;

            command(cmd);
            while (n-- > 0) {
                data(*table++);
            }
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

int LVGLDispILI9341::tft_init()
{
	hard_reset();
    _cs = 0;
    write_table(reset_off, sizeof(reset_off));
    // write_table(ILI9341_regValues_ada, sizeof(ILI9341_regValues_ada));   //can change PIXFMT
    // write_table(ILI9341_regValues_post, sizeof(ILI9341_regValues_post));
    write_table(wake_on, sizeof(wake_on));
    setRotation(2);
    _cs = 1;

    _backlight =1;
	
	return 0;
}

void LVGLDispILI9341::set_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    command(ILI9341_CASET); // Column addr set
    data(x0 >> 8);
    data(x0);     // XSTART 
    data(x1 >> 8);
    data(x1);     // XEND
    
    command(ILI9341_PASET); // Row addr set
    data(y0 >> 8);
    data(y0);     // YSTART
    data(y1 >> 8);
    data(y1);     // YEND

    command(ILI9341_RAMWR);
}
