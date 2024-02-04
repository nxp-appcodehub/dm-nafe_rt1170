/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_iomuxc.h"
#include "fsl_edma.h"
#include "fsl_lpuart.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"

#include "board.h"
#include "clock_config.h"

#include "nafe13388.h"
#include "nafe_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define READY_GPIO_BASE             GPIO3
#define READY_PIN_INDEX             6u
#define READY_PIN_FLEXIO_PIN_INDEX  7u
#define READY_GPIO_MUX_CFG          IOMUXC_GPIO_AD_07_GPIO_MUX3_IO06
#define READY_FLEXIO_MUX_CFG        IOMUXC_GPIO_AD_07_FLEXIO2_D07

#define READY_PIN_ISR               GPIO3_Combined_0_15_IRQHandler
#define READY_PIN_IRQ_NUMBER        GPIO3_Combined_0_15_IRQn

#define SYNC_GPIO_BASE              GPIO3
#define SYNC_PIN_INDEX              13u
#define CS_GPIO_BASE                GPIO3
#define CS_PIN_INDEX                28u

#define GPIO_CLOCK_GATE             kCLOCK_Gpio

#define SPI_INSTANCE                LPSPI1
#define SPI_ISR                     LPSPI1_IRQHandler
#define SPI_IRQ_NUMBER              LPSPI1_IRQn
#define SPI_CLOCK_GATE              kCLOCK_Lpspi1

#define SPI_READ_DMA_IRQ_NUMBER     DMA2_DMA18_IRQn
#define SPI_READ_DMA_ISR            DMA2_DMA18_IRQHandler

#define FLEXIO_INSTANCE             FLEXIO2
#define FLEXIO_CLOCK_GATE           kCLOCK_Flexio2
#define FLEXIO_TIMER_INDEX          0u
#define FLEXIO_SHIFTER_INDEX        0u

#define DMA_INSTANCE                DMA0
#define DMA_MUX_INSTANCE            DMAMUX0
#define DMA_CLOCK_GATE              kCLOCK_Edma

#define FLEXIO_DMA_REQUEST          kDmaRequestMuxFlexIO2Request0Request1
#define SPI_DMA_REQUEST             kDmaRequestMuxLPSPI1Rx
#define SPI_FLEXIO_DMA_CHN          0u
#define SPI_WRITE_DMA_CHN           1u
#define SPI_READ_DMA_CHN            2u

#define DEMO_MULTI_CHANNEL_AMT      2u
#define DEMO_SAMPLE_AMT             50u
#define DEMO_SAMPLE_MODE            kNafeSampleMode_mcmrNonBlock

/*******************************************************************************
 * Variables
 ******************************************************************************/
static NAFE_HAL_hdl_t gHalHdl = {
    .spiInst            = SPI_INSTANCE,
    .csGpioBase         = CS_GPIO_BASE,
    .csGpioIndex        = CS_PIN_INDEX,
    .readyPinGpioBase   = READY_GPIO_BASE,
    .readyPinGpioIndex  = READY_PIN_INDEX,
    .syncGpioBase       = SYNC_GPIO_BASE,
    .syncGpioIndex      = SYNC_PIN_INDEX,

    /* For DMA approaches. */
    .flexioInst         = FLEXIO_INSTANCE,
    .dmaInst            = DMA_INSTANCE,
    .dmamuxInst         = DMA_MUX_INSTANCE,
    .dmaChnRdyTrig      = SPI_FLEXIO_DMA_CHN,
    .dmaChnWrSpi        = SPI_WRITE_DMA_CHN,
    .dmaChnRdSpi        = SPI_READ_DMA_CHN,
    .flexioTimerIndex   = FLEXIO_TIMER_INDEX,
    .flexioShifterIndex = FLEXIO_SHIFTER_INDEX,
    .flexioPinIndex     = READY_PIN_FLEXIO_PIN_INDEX,
    .flexioRxDmaReqSrc  = (uint32_t)(FLEXIO_DMA_REQUEST & 0xFF),
    .spiRxDmaReqSrc     = (uint32_t)(SPI_DMA_REQUEST & 0xFF)
};

static NAFE_sysConfig_t gSysConfig = {
#if 0
    .adcResolutionCode = kNafeAdcResolution_16bits,
#else
    .adcResolutionCode = kNafeAdcResolution_24bits,
#endif

#if 0
    .triggerMode = kNafeTrigger_spiCmd,
#else
    .triggerMode = kNafeTrigger_syncPin,
#endif

#if 0
    .readyPinSeqMode = kNafeReadyPinSeqMode_onConversion,
#else
    .readyPinSeqMode = kNafeReadyPinSeqMode_onSequencer,
#endif
};

static NAFE_chnConfig_t gChnConfigArray[DEMO_MULTI_CHANNEL_AMT] = {
    {
        .chnIndex = 0u,                             /* The first channel number index */
        .inputSel = kNafeInputSel_hvsig,
        .hvAip = kNafeHvInputPos_ai1p,
        .hvAin = kNafeHvInputNeg_ai1n,
        .gain = kNafeChnGain_1x,
        .dataRateCode = 4u,
        .adcSinc = kNafeAdcSinc_sinc4,
        .chDelayCode = 0u,
        .adcSettling = kNafeAdcSettling_singleCycle,
        .viexVi = kNafeViexVi_voltage,
        .viexPol = kNafeViexPol_positive,
        .viexMag = kNafeViexMag_0mV_0uA,
        .viexChop = kNafeViexChop_disable,
        .viexAipEn = kNafeViexAipEn_none,
        .viexAinEn = kNafeViexAinEn_none
    },
    {
        .chnIndex = 2u,                             /* The second channel number index */
        .inputSel = kNafeInputSel_hvsig,
        .hvAip = kNafeHvInputPos_ai2p,
        .hvAin = kNafeHvInputNeg_ai2n,
        .gain = kNafeChnGain_0p4x,
        .dataRateCode = 4u,
        .adcSinc = kNafeAdcSinc_sinc4,
        .chDelayCode = 0u,
        .adcSettling = kNafeAdcSettling_singleCycle,
        .viexVi = kNafeViexVi_voltage,
        .viexPol = kNafeViexPol_positive,
        .viexMag = kNafeViexMag_0mV_0uA,
        .viexChop = kNafeViexChop_disable,
        .viexAipEn = kNafeViexAipEn_none,
        .viexAinEn = kNafeViexAinEn_none
    }
};

static NAFE_devHdl_t gDevHdl = {
    .devAddr = 0u,
    .sysConfig = &gSysConfig,
    .chConfig = gChnConfigArray,
#if 0
    .readResultFrameMode = kNafeReadResult_multiFrame,
#else
    .readResultFrameMode = kNafeReadResult_singleFrame,
#endif
    .currentSampleMode = kNafeSampleMode_none,
    .halHdl = &gHalHdl
};

static NAFE_xferHdl_t gXferHdl = {
    .sampleMode = kNafeSampleMode_none
};

static float gResultArray[DEMO_MULTI_CHANNEL_AMT][DEMO_SAMPLE_AMT];
static float gResultNonContinuous[DEMO_MULTI_CHANNEL_AMT];

static float gResultArrayView0[DEMO_SAMPLE_AMT];
static float gResultArrayView1[DEMO_SAMPLE_AMT];
static uint32_t gResultArrayViewTrigger = 0;

/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/
static void waveformDemoConfig(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);
static void waveformDemoRun(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);
static void init_freemaster_lpuart(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

void waveformDemoInit(void)
{
    CLOCK_EnableClock(SPI_CLOCK_GATE);
    CLOCK_EnableClock(GPIO_CLOCK_GATE);
    CLOCK_EnableClock(FLEXIO_CLOCK_GATE);
    CLOCK_EnableClock(DMA_CLOCK_GATE);

    init_freemaster_lpuart();
    FMSTR_Init();
}

void waveformDemo(void)
{
     /* Select the sample mode. */
    gXferHdl.sampleMode = DEMO_SAMPLE_MODE;

    /* Initialization and configuration. */
    waveformDemoConfig(&gDevHdl, &gXferHdl);

    while (1)
    {
        memset(gResultNonContinuous, 0, sizeof(gResultNonContinuous));
        memset(gResultArray, 0, sizeof(gResultArray));

        waveformDemoRun(&gDevHdl, &gXferHdl);
    }
}

static void waveformDemoConfig(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    xferHdl->blockingCallback = NULL;

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
            xferHdl->chnAmt = 1u;
            xferHdl->pResult = gResultNonContinuous;
            xferHdl->contSampleAmt = 1;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_sccrBlock:
            xferHdl->chnAmt = 1u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            xferHdl->blockingCallback = &FMSTR_Poll;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_mcmrBlock:
            xferHdl->chnAmt = 2u;
            xferHdl->pResult = gResultNonContinuous;
            xferHdl->contSampleAmt = 1;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_mccrBlock:
            xferHdl->chnAmt = 2u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            xferHdl->blockingCallback = &FMSTR_Poll;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_scsrNonBlock:
            xferHdl->chnAmt = 1u;
            xferHdl->pResult = gResultNonContinuous;
            xferHdl->contSampleAmt = 1;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_sccrNonBlock:
            xferHdl->chnAmt = 1u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_mcmrNonBlock:
            xferHdl->chnAmt = 2u;
            xferHdl->pResult = gResultNonContinuous;
            xferHdl->contSampleAmt = 1;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_mccrNonBlock:
            xferHdl->chnAmt = 2u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            IOMUXC_SetPinMux(READY_GPIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_sccrDma:
            xferHdl->chnAmt = 1u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            devHdl->readResultFrameMode = kNafeReadResult_singleFrame;              /* For DMA approach: only this option is supported. */
            IOMUXC_SetPinMux(READY_FLEXIO_MUX_CFG, 0U);
            break;
        case kNafeSampleMode_mccrDma:
            xferHdl->chnAmt = 2u;
            xferHdl->pResult = gResultArray;
            xferHdl->contSampleAmt = DEMO_SAMPLE_AMT;
            devHdl->sysConfig->readyPinSeqMode = kNafeReadyPinSeqMode_onSequencer;  /* For DMA approach: only this option is supported. */
            devHdl->readResultFrameMode = kNafeReadResult_singleFrame;              /* For DMA approach: only this option is supported. */
            IOMUXC_SetPinMux(READY_FLEXIO_MUX_CFG, 0U);
            break;
        default:
            break;
    }

    NAFE_init(devHdl, xferHdl);

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
            /* Enable SPI and Ready Pin IRQ For interrupt methods. */
            NVIC_ClearPendingIRQ(SPI_IRQ_NUMBER);
            NVIC_ClearPendingIRQ(READY_PIN_IRQ_NUMBER);
            NVIC_EnableIRQ(SPI_IRQ_NUMBER);
            NVIC_EnableIRQ(READY_PIN_IRQ_NUMBER);
            break;
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            /* Enable DMA permission and IRQ for DMA methods. */
            NVIC_EnableIRQ(SPI_READ_DMA_IRQ_NUMBER);
            break;
        default:
            break;
    }
}

static void waveformDemoRun(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    uint32_t nonContSampleCnt;    /* Non-continuous sample counter. For SCSR and MCMR modes. */

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
        case kNafeSampleMode_mcmrBlock:
            nonContSampleCnt = 0;
            while (nonContSampleCnt < DEMO_SAMPLE_AMT)
            {
                NAFE_startSample(devHdl, xferHdl);
                for (uint32_t i = 0; i < xferHdl->chnAmt; i++)
                {
                    gResultArray[i][nonContSampleCnt] = gResultNonContinuous[i];
                }
                nonContSampleCnt++;
                FMSTR_Poll();
            }
            NAFE_terminateSample(devHdl, xferHdl);
            memcpy(gResultArrayView0, gResultArray[0], sizeof(float) * DEMO_SAMPLE_AMT);
            memcpy(gResultArrayView1, gResultArray[1], sizeof(float) * DEMO_SAMPLE_AMT);
            gResultArrayViewTrigger ^= 1;
            break;
        case kNafeSampleMode_sccrBlock:
        case kNafeSampleMode_mccrBlock:
            NAFE_startSample(devHdl, xferHdl);
            memcpy(gResultArrayView0, gResultArray[0], sizeof(float) * DEMO_SAMPLE_AMT);
            memcpy(gResultArrayView1, gResultArray[1], sizeof(float) * DEMO_SAMPLE_AMT);
            gResultArrayViewTrigger ^= 1;
            break;
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
            nonContSampleCnt = 0;
            while (nonContSampleCnt < DEMO_SAMPLE_AMT)
            {
                NAFE_startSample(devHdl, xferHdl);
                while (xferHdl->nonBlockXferState != kNafeNonBlockXferState_done)
                {
                }

                for (uint32_t i = 0; i < xferHdl->chnAmt; i++)
                {
                    gResultArray[i][nonContSampleCnt] = gResultNonContinuous[i];
                }
                nonContSampleCnt++;
                FMSTR_Poll();
            }
            NAFE_terminateSample(devHdl, xferHdl);
            memcpy(gResultArrayView0, gResultArray[0], sizeof(float) * DEMO_SAMPLE_AMT);
            memcpy(gResultArrayView1, gResultArray[1], sizeof(float) * DEMO_SAMPLE_AMT);
            gResultArrayViewTrigger ^= 1;
            FMSTR_Poll();
            break;
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            NAFE_startSample(devHdl, xferHdl);
            while (xferHdl->nonBlockXferState != kNafeNonBlockXferState_done)
            {
                FMSTR_Poll();
            }
            memcpy(gResultArrayView0, gResultArray[0], sizeof(float) * DEMO_SAMPLE_AMT);
            memcpy(gResultArrayView1, gResultArray[1], sizeof(float) * DEMO_SAMPLE_AMT);
            gResultArrayViewTrigger ^= 1;
            FMSTR_Poll();
            break;
        default:
            break;
    }
}

static void init_freemaster_lpuart(void)
{
    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = false;
    config.enableRx = false;

    LPUART_Init((LPUART_Type*)BOARD_DEBUG_UART_BASEADDR, &config, BOARD_DEBUG_UART_CLK_FREQ);

    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress((LPUART_Type*)BOARD_DEBUG_UART_BASEADDR);

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    /* Enable UART interrupts. */
    EnableIRQ(BOARD_UART_IRQ);
    EnableGlobalIRQ(0u);
#endif
}

void SPI_ISR(void)
{
    NAFE_irqHandle(&gDevHdl, &gXferHdl, kNafeInterrupt_spi);
}

void READY_PIN_ISR(void)
{
    NAFE_irqHandle(&gDevHdl, &gXferHdl, kNafeInterrupt_readyPin);
}

void SPI_READ_DMA_ISR(void)
{
    NAFE_dmaIrqHandle(&gDevHdl, &gXferHdl);
}
