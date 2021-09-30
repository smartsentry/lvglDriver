
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

#include "LVGLDispDriverSTM32F407VE_BLACK.h"
#include "ili9341_fsmc.h"

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)
#define USE_DMA                 (1)

/*
    use FSMC
*/

volatile uint16_t *ili9341_fsmcCommand;
volatile uint16_t *ili9341_fsmcData;

static int fsmc_lcd_init()
{
    // enable clocks
    // workaround, the HAL macros produce warnings
    volatile uint32_t dummy;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    dummy = RCC->AHB1ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    dummy = RCC->AHB1ENR;
    RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
    dummy = RCC->AHB3ENR;
    dummy = dummy;

    // set GPIO Alternate Function FSMC
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
                             GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                             GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                             GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

    FSMC_NORSRAM_InitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMInitStructure.NSBank = FSMC_NORSRAM_BANK1;                       // ??
    FSMC_NORSRAMInitStructure.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    FSMC_NORSRAMInitStructure.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    FSMC_NORSRAMInitStructure.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    FSMC_NORSRAMInitStructure.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;    // disable
    FSMC_NORSRAMInitStructure.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    FSMC_NORSRAMInitStructure.WrapMode = FSMC_WRAP_MODE_DISABLE;
    FSMC_NORSRAMInitStructure.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    FSMC_NORSRAMInitStructure.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    FSMC_NORSRAMInitStructure.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    FSMC_NORSRAMInitStructure.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    FSMC_NORSRAMInitStructure.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    FSMC_NORSRAMInitStructure.WriteBurst = FSMC_WRITE_BURST_DISABLE;  // enable?
    FSMC_NORSRAMInitStructure.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
    FSMC_NORSRAMInitStructure.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
    FSMC_NORSRAMInitStructure.PageSize = 0;

    HAL_StatusTypeDef status = HAL_OK;
    status = FSMC_NORSRAM_Init(FSMC_Bank1, &FSMC_NORSRAMInitStructure);

    FSMC_NORSRAM_TimingTypeDef FSMC_NORSRAMTimingInitStructure;
    FSMC_NORSRAMTimingInitStructure.AddressSetupTime = 7; // 100 ns (ADDSET+1)*12.5ns = CS to RW
    FSMC_NORSRAMTimingInitStructure.AddressHoldTime = 0;
    FSMC_NORSRAMTimingInitStructure.DataSetupTime = 3;    // 50 ns (DATAST+1)*12.5ns = RW length
    FSMC_NORSRAMTimingInitStructure.BusTurnAroundDuration = 0;
    FSMC_NORSRAMTimingInitStructure.CLKDivision = 0;
    FSMC_NORSRAMTimingInitStructure.DataLatency = 0;
    FSMC_NORSRAMTimingInitStructure.AccessMode = FSMC_ACCESS_MODE_A;
    FSMC_NORSRAM_Init(FSMC_Bank1, &FSMC_NORSRAMInitStructure);

    status = FSMC_NORSRAM_Timing_Init(FSMC_Bank1,  &FSMC_NORSRAMTimingInitStructure, FSMC_NORSRAM_BANK1);

    /* Enable FSMC NOR/SRAM Bank1 */
    __FSMC_NORSRAM_ENABLE(FSMC_Bank1, FSMC_NORSRAM_BANK1);

    ili9341_fsmcCommand    = (volatile uint16_t *)NOR_MEMORY_ADRESS1;   // clears A18
    ili9341_fsmcData       = (ili9341_fsmcCommand + (1 << 18));                 // sets A18

    return status;
};

static DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

extern "C" void dmaXFerComplete(DMA_HandleTypeDef *hdma) 
{
    if (hdma == &hdma_memtomem_dma2_stream0)
	{
        lv_disp_flush_ready(LVGLDispDriver::get_target_default_instance()->getLVDispDrv());
    }
}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void fsmc_dma_init(void) 
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
    hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
    hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_DISABLE;
    hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
    hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
    {
        debug("dma init error\n");
    }

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // register DMA-Callback
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0,
			HAL_DMA_XFER_CPLT_CB_ID, &dmaXFerComplete);
}


LVGLDispSTM32F407VE_BLACK::LVGLDispSTM32F407VE_BLACK(uint32_t nBufferRows) :
    LVGLDispDriver(LV_HOR_RES_MAX, LV_VER_RES_MAX),
    _nBufferRows(nBufferRows)
{
    // low level hardware init
    fsmc_lcd_init();
    fsmc_dma_init();

    // tft controller init
    ili9341_fsmc_init();
    ili9341_fsmc_setRotation(1);

    init();
}

void LVGLDispSTM32F407VE_BLACK::init()
{
    size_t bufferSize = LV_HOR_RES_MAX * _nBufferRows;

    // allocate memory for display buffer
    _buf1_1 = new lv_color_t[bufferSize];             /* a buffer for n rows */
    MBED_ASSERT(_buf1_1 != nullptr);
    memset(_buf1_1, 0, bufferSize*sizeof(lv_color_t));

    /*Used to copy the buffer's content to the display*/
    _disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    _disp_drv.draw_buf = &_disp_buf_1;

    lv_disp_draw_buf_init(&_disp_buf_1, _buf1_1, NULL, bufferSize);   /* Initialize the display buffer */

    /*Finally register the driver*/
    _disp_drv.user_data = this;
    _disp = lv_disp_drv_register(&_disp_drv);
}

void LVGLDispSTM32F407VE_BLACK::disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    ili9341_fsmc_setAddrWindow(area->x1, area->y1, area->x2, area->y2);
    *ili9341_fsmcCommand = ILI9341_MEMORYWRITE; // 0x2c
#ifdef USE_DMA
    uint32_t width = area->x2 - area->x1 + 1;
    uint32_t height =  area->y2 - area->y1 + 1;

	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,
			(uint32_t)color_p, (uint32_t)ili9341_fsmcData, width*height);
#else
    for (int32_t y = area->y1; y <= area->y2; y++) {
        for (int32_t x = area->x1; x <= area->x2; x++) {
            *ili9341_fsmcData = color_p->full;
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
    static LVGLDispSTM32F407VE_BLACK drv;
    return &drv;
}

extern "C" void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream0);
}
