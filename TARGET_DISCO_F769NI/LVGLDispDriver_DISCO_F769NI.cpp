
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

/*
 * Tear-free display driver for the STM32F769I-DISCO (DSI video mode).
 *
 * Two independent causes of tearing are handled here:
 *
 * 1. Panel-side (the diagonal tear): when the panel controller is put into
 *    landscape with MADCTL, it writes the incoming video stream into its GRAM
 *    row-wise but still scans the GRAM out in its native portrait direction.
 *    The two pointers cross diagonally. The BSP therefore now initialises the
 *    panel in native PORTRAIT (480x800) and this driver rotates LVGL's
 *    landscape (800x480) output by 90 degrees in software while flushing.
 *    LVGL itself still sees an unrotated 800x480 display, so touch input and
 *    the UI are unchanged.
 *
 * 2. LTDC-side: the LTDC continuously scans the active framebuffer, so it must
 *    never be written while visible. Two full portrait framebuffers are used in
 *    SDRAM; LVGL's partial chunks are rotated into the hidden (back) buffer and
 *    the LTDC layer address is swapped during vertical blanking. The LTDC
 *    register-reload interrupt then reports lv_display_flush_ready().
 *    Before the next frame is drawn, the areas changed in the previous frame
 *    are copied (DMA2D) from the front into the back buffer so it is complete.
 */

#include "LVGLDispDriver_DISCO_F769NI.h"
#include "src/display/lv_display_private.h"     // inv_areas: skip needless buffer syncs
#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "LCD"

/* Logical (LVGL) resolution: landscape */
#define LV_HOR_RES_MAX          (800)
#define LV_VER_RES_MAX          (480)

/* Physical (panel native) resolution: portrait */
#define PHYS_W                  (480)
#define PHYS_H                  (800)

/* Rotation direction from LVGL landscape to panel portrait.
 * 1: clockwise (matches the BSP's former MADCTL landscape setting).
 * Set to 0 if the picture appears upside down. */
#ifndef DISPLAY_ROTATE_CW
#define DISPLAY_ROTATE_CW       (1)
#endif

/* Two ARGB8888 portrait framebuffers in SDRAM, 2 MB apart (each needs 1.5 MB).
 * FB0 is the one the BSP initialises the LTDC layer with (LCD_FB_START_ADDRESS). */
#define FB_BYTES                (PHYS_W * PHYS_H * 4)
#define FB0_ADDRESS             (LCD_FB_START_ADDRESS)
#define FB1_ADDRESS             (LCD_FB_START_ADDRESS + 0x00200000UL)

/* LVGL partial render buffer (logical landscape rows) in SDRAM after the FBs. */
#define DRAWBUF_ADDRESS         (LCD_FB_START_ADDRESS + 0x00400000UL)
#define DRAWBUF_MAX_BYTES       (0x00100000UL)

/* MPU regions (0..3 are used by Mbed) */
#define FB_MPU_REGION           (4)     // FB0+FB1: write-through (read by LTDC/DMA2D)
#define DRAWBUF_MPU_REGION      (5)     // draw buffer: write-back (CPU only)

/* Max. number of areas remembered for the front->back buffer sync */
#define MAX_SYNC_AREAS          (LV_INV_BUF_SIZE)

extern LTDC_HandleTypeDef  hltdc_discovery;
extern DMA2D_HandleTypeDef hdma2d_discovery;

LVGLDispDISCO_F769NI *display;

/* Buffer state. s_front is on screen, s_back is being drawn. */
static volatile uint32_t s_front = FB0_ADDRESS;
static volatile uint32_t s_back  = FB1_ADDRESS;
static bool              s_frame_open = false;
static lv_area_t         s_prev_areas[MAX_SYNC_AREAS];     // logical areas changed in last frame
static uint32_t          s_prev_cnt = 0;

/* DMA2D IRQ vector is installed by BSP_LCD_MspInit(). DMA2D is used here in
 * polling mode only, so just acknowledge any flags. */
extern "C" void dmaXFerComplete()
{
    WRITE_REG(hdma2d_discovery.Instance->IFCR,
              DMA2D_IFCR_CTEIF | DMA2D_IFCR_CTCIF | DMA2D_IFCR_CTWIF |
              DMA2D_IFCR_CAECIF | DMA2D_IFCR_CCTCIF | DMA2D_IFCR_CCEIF);
}

/* LTDC interrupt: the register reload (layer address swap) has taken effect
 * during vertical blanking -> the new frame is on screen, LVGL may continue. */
static void ltdc_irq_handler()
{
    uint32_t isr = LTDC->ISR;
    LTDC->ICR = LTDC_ICR_CLIF | LTDC_ICR_CFUIF | LTDC_ICR_CTERRIF | LTDC_ICR_CRRIF;

    if (isr & LTDC_ISR_RRIF) {
        LTDC->IER &= ~LTDC_IER_RRIE;
        uint32_t shown = s_back;
        s_back  = s_front;
        s_front = shown;
        if (display) {
            lv_display_flush_ready(display->getLVDisp());
        }
    }
}

/* Logical landscape area -> physical portrait area */
static void to_phys(const lv_area_t *a, lv_area_t *p)
{
#if DISPLAY_ROTATE_CW
    p->x1 = (PHYS_W - 1) - a->y2;
    p->x2 = (PHYS_W - 1) - a->y1;
    p->y1 = a->x1;
    p->y2 = a->x2;
#else
    p->x1 = a->y1;
    p->x2 = a->y2;
    p->y1 = (PHYS_H - 1) - a->x2;
    p->y2 = (PHYS_H - 1) - a->x1;
#endif
}

static bool area_contains(const lv_area_t *holder, const lv_area_t *a)
{
    return a->x1 >= holder->x1 && a->y1 >= holder->y1 &&
           a->x2 <= holder->x2 && a->y2 <= holder->y2;
}

/* Copy a physical area between framebuffers with DMA2D (memory-to-memory, polling) */
static void fb_copy_area(uint32_t src_fb, uint32_t dst_fb, const lv_area_t *p)
{
    uint32_t w = p->x2 - p->x1 + 1;
    uint32_t h = p->y2 - p->y1 + 1;
    uint32_t offset = (p->y1 * PHYS_W + p->x1) * 4;

    while (DMA2D->CR & DMA2D_CR_START) {}

    DMA2D->CR      = 0;                         // mode: memory-to-memory, no interrupts
    DMA2D->FGMAR   = src_fb + offset;
    DMA2D->FGOR    = PHYS_W - w;
    DMA2D->FGPFCCR = 0;                         // ARGB8888
    DMA2D->OMAR    = dst_fb + offset;
    DMA2D->OOR     = PHYS_W - w;
    DMA2D->OPFCCR  = 0;                         // ARGB8888
    DMA2D->NLR     = (w << DMA2D_NLR_PL_Pos) | (h << DMA2D_NLR_NL_Pos);
    DMA2D->IFCR    = DMA2D_IFCR_CTEIF | DMA2D_IFCR_CTCIF | DMA2D_IFCR_CTWIF |
                     DMA2D_IFCR_CAECIF | DMA2D_IFCR_CCTCIF | DMA2D_IFCR_CCEIF;
    DMA2D->CR     |= DMA2D_CR_START;

    while (DMA2D->CR & DMA2D_CR_START) {}
}

/* Bring the back buffer up to date with the front buffer: copy everything that
 * changed in the previous frame, except areas that this frame redraws anyway. */
static void sync_back_buffer(lv_display_t *disp)
{
    for (uint32_t i = 0; i < s_prev_cnt; i++) {
        bool covered = false;
        for (uint32_t j = 0; j < disp->inv_p && !covered; j++) {
            if (!disp->inv_area_joined[j] && area_contains(&disp->inv_areas[j], &s_prev_areas[i])) {
                covered = true;
            }
        }
        if (!covered) {
            lv_area_t p;
            to_phys(&s_prev_areas[i], &p);
            fb_copy_area(s_front, s_back, &p);
        }
    }
    s_prev_cnt = 0;
}

/* Remember the areas redrawn in this frame for the next sync */
static void save_frame_areas(lv_display_t *disp)
{
    s_prev_cnt = 0;
    for (uint32_t j = 0; j < disp->inv_p; j++) {
        if (disp->inv_area_joined[j]) continue;
        if (s_prev_cnt == MAX_SYNC_AREAS) {        // overflow: sync the whole screen
            lv_area_set(&s_prev_areas[0], 0, 0, LV_HOR_RES_MAX - 1, LV_VER_RES_MAX - 1);
            s_prev_cnt = 1;
            return;
        }
        s_prev_areas[s_prev_cnt++] = disp->inv_areas[j];
    }
}

LVGLDispDISCO_F769NI::LVGLDispDISCO_F769NI(uint32_t nBufferRows) :
    LVGLDispDriver(LV_HOR_RES_MAX, LV_VER_RES_MAX),
    _nBufferRows(nBufferRows)
{
    // init display (BSP constructor of _lcd, panel in portrait) and clear FB0
    _lcd.Clear(LCD_COLOR_BLACK);
    _lcd.SetBackColor(LCD_COLOR_BLACK);
    _lcd.SetTextColor(LCD_COLOR_WHITE);

    display = this;
    init();
}

void LVGLDispDISCO_F769NI::init()
{
    MBED_ASSERT(BSP_LCD_GetXSize() == PHYS_W && BSP_LCD_GetYSize() == PHYS_H);

    size_t drawBufBytes = LV_HOR_RES_MAX * _nBufferRows * 4;
    if (drawBufBytes > DRAWBUF_MAX_BYTES) {
        drawBufBytes = (DRAWBUF_MAX_BYTES / (LV_HOR_RES_MAX * 4)) * (LV_HOR_RES_MAX * 4);
    }

    HAL_MPU_Disable();
    // Framebuffers: normal memory, write-through, not shareable. The CPU only
    // writes them (rotation), the LTDC and DMA2D read them -> always coherent.
    ARM_MPU_SetRegion(
        ARM_MPU_RBAR(FB_MPU_REGION, FB0_ADDRESS),
        ARM_MPU_RASR(1, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_4MB));
    // Draw buffer: normal memory, write-back write-allocate, CPU access only.
    ARM_MPU_SetRegion(
        ARM_MPU_RBAR(DRAWBUF_MPU_REGION, DRAWBUF_ADDRESS),
        ARM_MPU_RASR(1, ARM_MPU_AP_FULL, 1, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_1MB));
    HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
    SCB_CleanInvalidateDCache();
    tr_info("MPU regions %d/%d set: FB %p..%p, draw buffer %p (%u bytes)",
            FB_MPU_REGION, DRAWBUF_MPU_REGION, (void *)FB0_ADDRESS,
            (void *)(FB1_ADDRESS + FB_BYTES - 1), (void *)DRAWBUF_ADDRESS, (unsigned)drawBufBytes);

    // FB0 is on screen and already cleared by the BSP; make FB1 identical
    lv_area_t full = { 0, 0, PHYS_W - 1, PHYS_H - 1 };
    fb_copy_area(FB0_ADDRESS, FB1_ADDRESS, &full);

    // Use constant alpha only, so the (unused) alpha byte of LVGL's XRGB8888
    // pixels can never make the layer transparent
    hltdc_discovery.LayerCfg[0].BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    hltdc_discovery.LayerCfg[0].BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    hltdc_discovery.LayerCfg[0].FBStartAdress   = FB0_ADDRESS;
    HAL_LTDC_ConfigLayer(&hltdc_discovery, &hltdc_discovery.LayerCfg[0], 0);
    s_front = FB0_ADDRESS;
    s_back  = FB1_ADDRESS;

    // LTDC interrupt: used for the vsync-synchronised buffer swap
    NVIC_SetVector(LTDC_IRQn, (uint32_t)&ltdc_irq_handler);
    HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);

    _disp = lv_display_create(LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_display_set_flush_cb(_disp, disp_flush);
    lv_display_set_user_data(_disp, this);
    lv_display_set_buffers(_disp, (void *)DRAWBUF_ADDRESS, nullptr, drawBufBytes,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    tr_info("display: portrait panel, sw rotation, double framebuffer, vsync swap");
}

void LVGLDispDISCO_F769NI::disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t *px_map)
{
    // First chunk of a new frame: the previous swap is complete (LVGL waited
    // for flush_ready), so bring the back buffer up to date first.
    if (!s_frame_open) {
        sync_back_buffer(disp);
        s_frame_open = true;
    }

    // Rotate the rendered landscape chunk into the portrait back buffer
    lv_area_t p;
    to_phys(area, &p);
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);
    uint8_t *dest = (uint8_t *)s_back + (p.y1 * PHYS_W + p.x1) * 4;

    lv_draw_sw_rotate(px_map, dest, w, h,
                      lv_draw_buf_width_to_stride(w, LV_COLOR_FORMAT_XRGB8888),
                      PHYS_W * 4,
#if DISPLAY_ROTATE_CW
                      LV_DISPLAY_ROTATION_270,      // LVGL's 270 == clockwise
#else
                      LV_DISPLAY_ROTATION_90,
#endif
                      LV_COLOR_FORMAT_XRGB8888);

    if (!lv_display_flush_is_last(disp)) {
        lv_display_flush_ready(disp);
        return;
    }

    // Frame complete: remember what changed, then show it at the next vblank
    save_frame_areas(disp);
    s_frame_open = false;
    __DSB();                                   // drain write buffer to SDRAM

    LTDC_LAYER(&hltdc_discovery, 0)->CFBAR = s_back;
    hltdc_discovery.LayerCfg[0].FBStartAdress = s_back;     // keep BSP draw functions in sync

    LTDC->ICR  = LTDC_ICR_CRRIF;
    LTDC->IER |= LTDC_IER_RRIE;
    LTDC->SRCR = LTDC_SRCR_VBR;
    // lv_display_flush_ready() and the front/back swap follow in ltdc_irq_handler()
}

MBED_WEAK LVGLDispDriver *LVGLDispDriver::get_target_default_instance()
{
    static LVGLDispDISCO_F769NI drv;
    return &drv;
}
