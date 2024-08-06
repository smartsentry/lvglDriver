#include "LVGLDispDriver_ILI9341.h"


class KD024QVRMA041: public LVGLDispILI9341{
    public:
    KD024QVRMA041(SPI &spi, PinName pinCS, PinName pinCMD, PinName pinRST, PinName pinBacklight, 
                               uint32_t nBufferRows, uint32_t resolutionX, uint32_t resolutionY);
    private:
//     int tft_init();

//     static constexpr  uint8_t init_cmd_list[] = {
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

};