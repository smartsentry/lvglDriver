
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

#include "LVGLDispDriver_DISCO_F769NI.h"
// #include "systool.h"

#define LV_HOR_RES_MAX          (800)
#define LV_VER_RES_MAX          (480)
#define BUFFERLINES (20)
#define USE_STATIC_BUFFER (0)
#define USE_DMA (1)
#define USE_DOUBLE_BUFFER (1)

static Timer t;
static int t0;
static int t1;

#if (USE_STATIC_BUFFER == 1)
static lv_color_t xbuf1[LV_HOR_RES_MAX * BUFFERLINES];
#  if   (USE_DOUBLE_BUFFER == 1)
static lv_color_t xbuf2[LV_HOR_RES_MAX * BUFFERLINES];
#  endif
#endif

#if (USE_DMA == 1)
extern "C" void dmaXFerComplete(DMA2D_HandleTypeDef *hdma2d) 
{
    lv_disp_flush_ready(LVGLDispDriver::get_target_default_instance()->getLVDispDrv());
    // t1 = t.elapsed_time().count();      // get µs timestamp
}
#else
    LCD_DISCO_F769NI* pLcd;
#endif

LVGLDispDISCO_F769NI::LVGLDispDISCO_F769NI(uint32_t nBufferRows) :
    LVGLDispDriver(LV_HOR_RES_MAX, LV_VER_RES_MAX),
    _nBufferRows(nBufferRows)
{
    // init display and show message using BSP draw functions (not lvgl)
    _lcd.Clear(LCD_COLOR_BLUE);
    _lcd.SetBackColor(LCD_COLOR_BLUE);
    _lcd.SetTextColor(LCD_COLOR_WHITE);
    _lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"DISCOVERY STM32F769NI", CENTER_MODE);

    init();
}

void LVGLDispDISCO_F769NI::init()
{
    size_t bufferSize = LV_HOR_RES_MAX * _nBufferRows;

#if (USE_STATIC_BUFFER == 0)
    // allocate memory for display buffer
    lv_color_t* xbuf1 = new lv_color_t[bufferSize];          // buffer on heap
    // lv_color_t* xbuf1 = (lv_color_t*)0x20000800;             // buffer in DTCM
    //lv_color_t* xbuf1 = (lv_color_t*)0x20060000;             // buffer in SRAM1

    MBED_ASSERT(xbuf1 != nullptr);
    memset(xbuf1, 0, bufferSize*sizeof(lv_color_t));
    // debug("init display, using heap buffer, addr: %p  %s\n", xbuf1, get_RAM_name(xbuf1));
#  if   (USE_DOUBLE_BUFFER == 1)
    lv_color_t* xbuf2 = new lv_color_t[bufferSize];          // buffer on heap
    // lv_color_t* xbuf2 = (lv_color_t*)0x20010400;             // buffer in DTCM
    //lv_color_t* xbuf2 = (lv_color_t*)0x20070000;             // buffer in SRAM1

    MBED_ASSERT(xbuf2 != nullptr);
    memset(xbuf2, 0, bufferSize*sizeof(lv_color_t));
    // debug("init display, using double buffer, addr: %p  %s\n", xbuf2, get_RAM_name(xbuf2));
#  endif
#else
    // debug("init display, using static buffer, addr: %p  %s\n", xbuf1, get_RAM_name(xbuf1));
#  if   (USE_DOUBLE_BUFFER == 1)
    // debug("init display, using double buffer, addr: %p  %s\n", xbuf2, get_RAM_name(xbuf2));
#  endif
#endif

    /*Used to copy the buffer's content to the display*/
    _disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    _disp_drv.draw_buf = &_disp_buf_1;

#if (USE_DOUBLE_BUFFER == 1)
    lv_disp_draw_buf_init(&_disp_buf_1, xbuf1, xbuf2, bufferSize);   /* Initialize the display buffer */
#else
    lv_disp_draw_buf_init(&_disp_buf_1, xbuf1, nullptr, bufferSize);   /* Initialize the display buffer */
#endif

    /*Finally register the driver*/
    _disp = lv_disp_drv_register(&_disp_drv);

#if (USE_DMA == 1)
    debug("display using DMA\n");
    BSP_LCD_SetDMACpltCallback(dmaXFerComplete);

#if 1
    // configure buffer as write-through
    {
        /* Disable the MPU */
        HAL_MPU_Disable();

        ARM_MPU_SetRegion(
            ARM_MPU_RBAR(
                4,                          // Region 4: not (yet) used by Mbed
                (uint32_t)xbuf1),                // Base, must be aligned 
            ARM_MPU_RASR(
                1,                          // DisableExec
                ARM_MPU_AP_FULL,            // AccessPermission
                0,                          // TypeExtField

                1,                          // IsShareable
                1,                          // IsCacheable
                0,                          // IsBufferable
                
                0,                          // subregions
                ARM_MPU_REGION_SIZE_128KB)  // Size
        );

        /* Enable the MPU */
        HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
    }
#endif

#else
    pLcd = &_lcd;
    debug("display using DrawPixel, addr LVGLDispDISCO_F769NI: %p  %s\n", pLcd, get_RAM_name(pLcd));
#endif

    t.start();      // start performance measurement timer
}

void LVGLDispDISCO_F769NI::disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
#if (USE_DMA == 1)
    uint32_t width = area->x2 - area->x1 + 1;
    uint32_t height =  area->y2 - area->y1 + 1;

    // t0 = t.elapsed_time().count();
    // SCB_CleanDCache();  // ok, works
    //SCB_CleanDCache_by_Addr((uint32_t*)color_p, height * LV_HOR_RES_MAX * sizeof(lv_color_t)); // ok, works, but maybe slower than CleanDCache()

    // printf("transfer cache: %d us\n", t1-t0);   // print previous transfer time
    // t0 = t.elapsed_time().count();              // get start time in µs

    BSP_LCD_TransferBitmap(area->x1, area->y1, width, height, (uint32_t*)color_p);
#else
    // use DrawPixel to buffer write to screen
    int32_t x;
    int32_t y;
    for (y = area->y1; y <= area->y2; y++) {
        for (x = area->x1; x <= area->x2; x++) {
            pLcd->DrawPixel(x, y, *((uint32_t *)color_p));
            color_p++;
        }
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
#endif

}

MBED_WEAK LVGLDispDriver *LVGLDispDriver::get_target_default_instance()
{
    static LVGLDispDISCO_F769NI drv(BUFFERLINES);
    return &drv;
}