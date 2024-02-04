/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_iomuxc.h"
#include "fsl_edma.h"
#include "fsl_debug_console.h"

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

#define TERMINAL_COLOR_RED              "\033[0;31m"
#define TERMINAL_COLOR_BOLD_RED         "\033[1;31m"
#define TERMINAL_COLOR_GREEN            "\033[0;32m"
#define TERMINAL_COLOR_BOLD_GREEN       "\033[1;32m"
#define TERMINAL_COLOR_YELLOW           "\033[0;33m"
#define TERMINAL_COLOR_BOLD_YELLOW      "\033[1;33m"
#define TERMINAL_COLOR_BLUE             "\033[0;34m"
#define TERMINAL_COLOR_BOLD_BLUE        "\033[1;34m"
#define TERMINAL_COLOR_MAGENTA          "\033[0;35m"
#define TERMINAL_COLOR_BOLD_MAGENTA     "\033[1;35m"
#define TERMINAL_COLOR_CYAN             "\033[0;36m"
#define TERMINAL_COLOR_BOLD_CYAN        "\033[1;36m"
#define TERMINAL_COLOR_RESET            "\033[0m"
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

const static char * modeMenu[][3] = {
    { "1", " SCSR Block     ", "(Single-Channel Single-Reading      with Polling method)"},
    { "2", " SCCR Block     ", "(Single-Channel Continuous-Reading  with Polling method)"},
    { "3", " MCMR Block     ", "(Multi -Channel Multi-Reading       with Polling method)"},
    { "4", " MCCR Block     ", "(Multi -Channel Continuous-Reading  with Polling method)"},
    { "5", " SCSR Non-Block ", "(Single-Channel Single-Reading      with Interrupt method)"},
    { "6", " SCCR Non-Block ", "(Single-Channel Continuous-Reading  with Interrupt method)"},
    { "7", " MCMR Non-Block ", "(Multi -Channel Multi-Reading       with Interrupt method)"},
    { "8", " MCCR Non-Block ", "(Multi -Channel Continuous-Reading  with Interrupt method)"},
    { "9", " SCCR DMA       ", "(Single-Channel Continuous-Reading  with DMA method)"},
    { "a", " MCCR DMA       ", "(Multi -Channel Continuous-Reading  with DMA method)"},
};
/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/
static void terminalDemoSelectSampleMode(NAFE_xferHdl_t *xferHdl);
static void terminalDemoConfig(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);
static void terminalDemoRun(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);
static void terminalDemoPrintContResults(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

/*******************************************************************************
 * Functions
 ******************************************************************************/

void terminalDemoInit(void)
{
    CLOCK_EnableClock(SPI_CLOCK_GATE);
    CLOCK_EnableClock(GPIO_CLOCK_GATE);
    CLOCK_EnableClock(FLEXIO_CLOCK_GATE);
    CLOCK_EnableClock(DMA_CLOCK_GATE);
}

void terminalDemo(void)
{
     /* Select the sample mode. */
    terminalDemoSelectSampleMode(&gXferHdl);

    /* Initialization and configuration. */
    PRINTF(">> Configuring the sample mode...\r\n");
    terminalDemoConfig(&gDevHdl, &gXferHdl);

    while (1)
    {
        memset(gResultNonContinuous, 0, sizeof(gResultNonContinuous));
        memset(gResultArray, 0, sizeof(gResultArray));

        PRINTF(">> Ready to run. Press any key to start, except '"\
                TERMINAL_COLOR_BOLD_RED "q" TERMINAL_COLOR_RESET "' to quit.\r\n");
        char c = GETCHAR();
        if ('q' == c || 'Q' == c)
        {
            PRINTF("\r\n");
            return;
        }

        PRINTF(">> Sample in progress...");
        terminalDemoRun(&gDevHdl, &gXferHdl);

        PRINTF(" Done. The conversion results:\r\n");
        terminalDemoPrintContResults(&gDevHdl, &gXferHdl);
    }
}

static void terminalDemoSelectSampleMode(NAFE_xferHdl_t *xferHdl)
{
    while (1)
    {
        uint32_t modeAmt = sizeof(modeMenu)/sizeof(modeMenu[0]);
        char mode;
        uint32_t modeIndex;

        PRINTF(">> Select a sample mode by entering the index number:\r\n");
        for(uint32_t i = 0; i < modeAmt; i++)
        {
            PRINTF(TERMINAL_COLOR_BOLD_RED "   ");
            PRINTF(modeMenu[i][0]);
            PRINTF(TERMINAL_COLOR_RED);
            PRINTF(modeMenu[i][1]);
            PRINTF(TERMINAL_COLOR_YELLOW);
            PRINTF(modeMenu[i][2]);
            PRINTF("\r\n");
        }

        PRINTF(TERMINAL_COLOR_RESET ">> ");
        mode = GETCHAR();
        PRINTF(TERMINAL_COLOR_BOLD_RED);
        PUTCHAR(mode);
        PRINTF("\r\n" TERMINAL_COLOR_RESET);

        for(uint32_t i = 0; i < modeAmt; i++)
        {
            if (mode == modeMenu[i][0][0])
            {
                if (mode >= '1' && mode <='9')
                {
                    modeIndex = mode - '0';
                }
                else if (mode >= 'a' && mode <= 'z')
                {
                    modeIndex = mode - 'a' + 10u;
                }

                xferHdl->sampleMode = (NAFE_sampleMode_t)modeIndex;
                return;
            }
        }
        PRINTF("\r\n");
    }
}

static void terminalDemoConfig(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
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

static void terminalDemoRun(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
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
            }
            NAFE_terminateSample(devHdl, xferHdl);
            break;
        case kNafeSampleMode_sccrBlock:
        case kNafeSampleMode_mccrBlock:
            NAFE_startSample(devHdl, xferHdl);
            break;
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
            nonContSampleCnt = 0;
            while (nonContSampleCnt < DEMO_SAMPLE_AMT)
            {
                NAFE_startSample(devHdl, xferHdl);
                while (xferHdl->nonBlockXferState != kNafeNonBlockXferState_done)
                {}

                for (uint32_t i = 0; i < xferHdl->chnAmt; i++)
                {
                    gResultArray[i][nonContSampleCnt] = gResultNonContinuous[i];
                }
                nonContSampleCnt++;
            }
            NAFE_terminateSample(devHdl, xferHdl);
            break;
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            NAFE_startSample(devHdl, xferHdl);
            while (xferHdl->nonBlockXferState != kNafeNonBlockXferState_done)
            {}
            break;
        default:
            break;
    }
}

static void terminalDemoPrintContResults(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    float *pResult = (float *)gResultArray;

    for (uint32_t i = 0u; i < xferHdl->chnAmt; i++)
    {
        PRINTF(TERMINAL_COLOR_BOLD_GREEN);
        PRINTF("  Channel %d:\r\n", devHdl->chConfig[i].chnIndex);

        PRINTF(TERMINAL_COLOR_GREEN);
        for (uint32_t j = 0u; j < DEMO_SAMPLE_AMT; j++)
        {
            if (j % 5u == 0u)
            {
                PRINTF("    ");
            }

            PRINTF("%8.5f  ", (double)*pResult++);

            if ((j + 1u) % 5u == 0u)
            {
                PRINTF("\r\n");
            }
        }
    }
    PRINTF("\r\n" TERMINAL_COLOR_RESET);
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
