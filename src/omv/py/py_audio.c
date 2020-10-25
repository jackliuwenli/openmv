/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Audio Python module.
 */
#include <mp.h>
#include "systick.h"
#include "py_assert.h"
#include "py_helper.h"
#include "py/binary.h"
#include "pdm2pcm_glo.h"
#include "fb_alloc.h"
#include "omv_boardconfig.h"

#if MICROPY_PY_AUDIO

#define AUDIO_IN_SAI_PDMx_DMAx_CLK_ENABLE()         __HAL_RCC_BDMA_CLK_ENABLE()
#define AUDIO_IN_SAI_PDMx_DMAx_STREAM               BDMA_Channel1
#define AUDIO_IN_SAI_PDMx_DMAx_REQUEST              BDMA_REQUEST_SAI4_A
#define AUDIO_IN_SAI_PDMx_DMAx_IRQ                  BDMA_Channel1_IRQn
#define AUDIO_IN_SAI_PDMx_DMAx_IRQHandler           BDMA_Channel1_IRQHandler
#define AUDIO_IN_IRQ_PREPRIO                        ((uint32_t)0x0F)

#define OS_EXCEPTION(msg)   mp_obj_new_exception_msg(&mp_type_OSError, msg)
#define SAI_MIN(a,b)        ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

static SAI_HandleTypeDef hsai;
static DMA_HandleTypeDef hdma_sai_rx;

static const int n_channels = 2;
static PDM_Filter_Handler_t  PDM_FilterHandler;
static PDM_Filter_Config_t   PDM_FilterConfig;

typedef enum {
  DMA_XFER_NONE = (1 << 0),
  DMA_XFER_HALF = (1 << 1),
  DMA_XFER_FULL = (1 << 2),
} dma_xfer_status;

static volatile uint32_t xfer_status = 0;

#define PDM_BUFFER_SIZE     (128*2)
// BDMA can only access D3 SRAM4 memory.
uint16_t PDM_BUFFER[PDM_BUFFER_SIZE] __attribute__ ((aligned (32))) __attribute__((section(".d3_sram_buffer")));

void AUDIO_IN_SAI_PDMx_DMAx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hsai.hdmarx);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    xfer_status |= DMA_XFER_HALF;
    SCB_InvalidateDCache_by_Addr((uint32_t *)(&PDM_BUFFER[0]), PDM_BUFFER_SIZE / 2);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    xfer_status |= DMA_XFER_FULL;
    SCB_InvalidateDCache_by_Addr((uint32_t *)(&PDM_BUFFER[PDM_BUFFER_SIZE / 2]), PDM_BUFFER_SIZE / 2);
}

static mp_obj_t py_audio_init()
{
    hsai.Instance                    = AUDIO_SAI;
    hsai.Init.Protocol               = SAI_FREE_PROTOCOL;
    hsai.Init.AudioMode              = SAI_MODEMASTER_RX;
    hsai.Init.DataSize               = SAI_DATASIZE_16;
    hsai.Init.FirstBit               = SAI_FIRSTBIT_LSB;
    hsai.Init.ClockStrobing          = SAI_CLOCKSTROBING_RISINGEDGE;
    hsai.Init.Synchro                = SAI_ASYNCHRONOUS;
    hsai.Init.OutputDrive            = SAI_OUTPUTDRIVE_DISABLE;
    hsai.Init.NoDivider              = SAI_MASTERDIVIDER_DISABLE;
    hsai.Init.FIFOThreshold          = SAI_FIFOTHRESHOLD_1QF;
    hsai.Init.SynchroExt             = SAI_SYNCEXT_DISABLE;
    hsai.Init.AudioFrequency         = SAI_AUDIO_FREQUENCY_32K * 8; //2.048MHz
    hsai.Init.MonoStereoMode         = SAI_STEREOMODE;
    hsai.Init.CompandingMode         = SAI_NOCOMPANDING;
    hsai.Init.TriState               = SAI_OUTPUT_RELEASED;

    // The master clock output is DISABLED.
    // Used as reference clock for external decoders.
    hsai.Init.Mckdiv                 = 0;
    hsai.Init.MckOutput              = SAI_MCK_OUTPUT_DISABLE;
    hsai.Init.MckOverSampling        = SAI_MCK_OVERSAMPLING_DISABLE;

    // Enable and configure PDM mode.
    hsai.Init.PdmInit.Activation     = ENABLE;
    hsai.Init.PdmInit.MicPairsNbr    = 1;
    hsai.Init.PdmInit.ClockEnable    = SAI_PDM_CLOCK1_ENABLE;

    hsai.FrameInit.FrameLength       = 16;
    hsai.FrameInit.ActiveFrameLength = 1;
    hsai.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    hsai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_HIGH;
    hsai.FrameInit.FSOffset          = SAI_FS_FIRSTBIT;

    hsai.SlotInit.FirstBitOffset     = 0;
    hsai.SlotInit.SlotSize           = SAI_SLOTSIZE_DATASIZE;
    hsai.SlotInit.SlotNumber         = 1;
    hsai.SlotInit.SlotActive         = SAI_SLOTACTIVE_0;

    // Initialize the SAI
    HAL_SAI_DeInit(&hsai);
    if (HAL_SAI_Init(&hsai) != HAL_OK) {
        nlr_raise(OS_EXCEPTION("Failed to init SAI"));
    }

    // Enable the DMA clock
    AUDIO_IN_SAI_PDMx_DMAx_CLK_ENABLE();

    // Configure the SAI DMA
    hdma_sai_rx.Instance                 = AUDIO_IN_SAI_PDMx_DMAx_STREAM;
    hdma_sai_rx.Init.Request             = AUDIO_IN_SAI_PDMx_DMAx_REQUEST;
    hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    __HAL_LINKDMA(&hsai, hdmarx, hdma_sai_rx);
    
    // Initialize the DMA stream
    HAL_DMA_DeInit(&hdma_sai_rx);
    if (HAL_DMA_Init(&hdma_sai_rx) != HAL_OK) {
        nlr_raise(OS_EXCEPTION("SAI DMA init failed!"));
    }

    /* SAI DMA IRQ Channel configuration */
    NVIC_SetPriority(AUDIO_IN_SAI_PDMx_DMAx_IRQ, IRQ_PRI_DMA21);
    HAL_NVIC_EnableIRQ(AUDIO_IN_SAI_PDMx_DMAx_IRQ);

    // Init PDM library
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET;

    PDM_FilterHandler.bit_order  = PDM_FILTER_BIT_ORDER_MSB;
    PDM_FilterHandler.endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler.high_pass_tap = 2122358088;
    PDM_FilterHandler.out_ptr_channels = n_channels;
    PDM_FilterHandler.in_ptr_channels  = n_channels;
    PDM_Filter_Init(&PDM_FilterHandler);
    return mp_const_none;
}

static mp_obj_t py_audio_read_pdm(mp_obj_t buf_in)
{
    mp_buffer_info_t pdmbuf;
    mp_get_buffer_raise(buf_in, &pdmbuf, MP_BUFFER_WRITE);
    size_t typesize = mp_binary_get_size('@', pdmbuf.typecode, NULL);
    uint32_t xfer_samples = 0;
    uint32_t n_samples = pdmbuf.len / typesize;

    if (typesize != 2) {
        // Make sure the buffer is 16-Bits array.
        nlr_raise(OS_EXCEPTION("Wrong data type, expected 16-Bits array!"));
    }

    // Clear DMA buffer status
    xfer_status &= DMA_XFER_NONE;

    // Start DMA transfer
    if (HAL_SAI_Receive_DMA(&hsai, (uint8_t*) PDM_BUFFER, PDM_BUFFER_SIZE) != HAL_OK) {
        nlr_raise(OS_EXCEPTION("SAI DMA transfer failed!"));
    }

    while (n_samples) {
        uint32_t start = HAL_GetTick();
        // Wait for transfer complete.
        while ((xfer_status & DMA_XFER_FULL) == 0) {
            if ((HAL_GetTick() - start) >= 1000) {
                HAL_SAI_DMAStop(&hsai);
                nlr_raise(OS_EXCEPTION("SAI DMA transfer timeout!"));
            }
        }

        // Clear buffer state.
        xfer_status &= DMA_XFER_NONE;

        // Copy samples to pdm output buffer.
        uint32_t samples = SAI_MIN(n_samples, PDM_BUFFER_SIZE/2);
        for (int i=0; i<samples; i++, n_samples--, xfer_samples++) {
            ((uint16_t*)pdmbuf.buf)[xfer_samples] = PDM_BUFFER[i];
        }

        if (xfer_status & DMA_XFER_FULL) {
            printf("Dropping samples!\n");
        }
    }

    // Stop SAI DMA.
    HAL_SAI_DMAStop(&hsai);

    return mp_const_none;
}


static mp_obj_t py_audio_read_pcm(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    mp_buffer_info_t pcmbuf;
    mp_get_buffer_raise(args[0], &pcmbuf, MP_BUFFER_WRITE);
    int gain = py_helper_keyword_int(n_args, args, 1, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_gain), 24);

    size_t typesize = mp_binary_get_size('@', pcmbuf.typecode, NULL);
    uint32_t pcm_samples = pcmbuf.len / typesize;

    // Allocate PDM samples buffer.
    uint8_t *pdmbuf = NULL;
    uint32_t pdm_samples = pcm_samples * 128; //1024 bit / OSR 64 = 16 bits
    if (fb_avail() < (pdm_samples * 2)) { // 2 channels
        nlr_raise(OS_EXCEPTION("Failed to alloc temporary buffer!"));
    }
    fb_alloc_mark();
    pdmbuf = fb_alloc(pdm_samples * 2, FB_ALLOC_PREFER_SPEED);
    printf("buffer %p\n", pdmbuf);

    // Read PDM samples
    if (HAL_SAI_Receive(&hsai, (uint8_t*) pdmbuf, pdm_samples, 1000) != HAL_OK) {
        nlr_raise(OS_EXCEPTION("Failed read from SAI"));
    }

    // Convert PDM samples to PCM.
    PDM_FilterConfig.mic_gain = gain;
    PDM_FilterConfig.output_samples_number = pcm_samples / n_channels;
    PDM_FilterConfig.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig(&PDM_FilterHandler, &PDM_FilterConfig);
    for (int i=0; i<n_channels; i++) {
        PDM_Filter(&((uint8_t*)pdmbuf)[i], &((uint16_t*)pcmbuf.buf)[i], &PDM_FilterHandler);
    }

    fb_free();
    return mp_const_none;
}

static mp_obj_t py_audio_pdm_to_pcm(mp_obj_t buf_in, mp_obj_t buf_out)
{
    mp_buffer_info_t pdmbuf;
    mp_buffer_info_t pcmbuf;
    mp_get_buffer_raise(buf_in, &pdmbuf, MP_BUFFER_READ);
    mp_get_buffer_raise(buf_out, &pcmbuf, MP_BUFFER_WRITE);

    size_t typesize = mp_binary_get_size('@', pcmbuf.typecode, NULL);
    uint32_t pcm_samples = pcmbuf.len / typesize;

    // Convert PDM samples to PCM.
    PDM_FilterConfig.mic_gain = 24;
    PDM_FilterConfig.output_samples_number = pcm_samples / n_channels;
    PDM_FilterConfig.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig(&PDM_FilterHandler, &PDM_FilterConfig);
    for (int i=0; i<n_channels; i++) {
        PDM_Filter(&((uint8_t*)pdmbuf.buf)[i], &((uint16_t*)pcmbuf.buf)[i], &PDM_FilterHandler);
    }
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_audio_init_obj, py_audio_init);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_audio_read_pdm_obj, py_audio_read_pdm);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_audio_read_pcm_obj, 1, py_audio_read_pcm);
STATIC MP_DEFINE_CONST_FUN_OBJ_2(py_audio_pdm_to_pcm_obj, py_audio_pdm_to_pcm);

static const mp_map_elem_t globals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_audio) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_init),        (mp_obj_t)&py_audio_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_pcm),    (mp_obj_t)&py_audio_read_pcm_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_pdm),    (mp_obj_t)&py_audio_read_pdm_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pdm_to_pcm),  (mp_obj_t)&py_audio_pdm_to_pcm_obj },
};

STATIC MP_DEFINE_CONST_DICT(globals_dict, globals_dict_table);

const mp_obj_module_t audio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t)&globals_dict,
};

#endif //MICROPY_PY_AUDIO
