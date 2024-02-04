/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nafe_hal.h"
#include "fsl_flexio.h"

static void NAFE_HAL_initSpi(NAFE_HAL_hdl_t *halHdl);
static void NAFE_HAL_initSpiCsPin(NAFE_HAL_hdl_t *halHdl);
static void NAFE_HAL_initReadyPin(NAFE_HAL_hdl_t *halHdl);
static void NAFE_HAL_initSyncPin(NAFE_HAL_hdl_t *halHdl);
static void NAFE_HAL_initFlexIO(NAFE_HAL_hdl_t *halHdl);
static void NAFE_HAL_initDmaChannels(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl);

void NAFE_HAL_init(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl)
{
    NAFE_HAL_initSpiCsPin(halHdl);
    NAFE_HAL_initSpi(halHdl);
    NAFE_HAL_initSyncPin(halHdl);

    NAFE_HAL_configReadyPinInterrupt(halHdl, false);

    if (xferHdl->sampleMode == kNafeSampleMode_sccrDma \
     || xferHdl->sampleMode == kNafeSampleMode_mccrDma)
    {
        NAFE_HAL_initFlexIO(halHdl);
        NAFE_HAL_initDmaChannels(halHdl, xferHdl);
    }
    else
    {
        NAFE_HAL_initReadyPin(halHdl);
    }
}

static void NAFE_HAL_initSpi(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->spiInst->CR = LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK | LPSPI_CR_DBGEN_MASK;
    halHdl->spiInst->CR |= LPSPI_CR_RST_MASK;
    halHdl->spiInst->CR &= ~LPSPI_CR_RST_MASK;
    halHdl->spiInst->CR |= LPSPI_CR_MEN_MASK;

    halHdl->spiInst->CFGR1 = LPSPI_CFGR1_MASTER_MASK;
    halHdl->spiInst->CCR = LPSPI_CCR_SCKPCS(3u) | LPSPI_CCR_PCSSCK(3u) \
                         | LPSPI_CCR_DBT(10u) | LPSPI_CCR_SCKDIV(10u); /* 24MHz/(10+2) = 2MHz baud rate */

    halHdl->spiTcrBaseline = LPSPI_TCR_CPOL(0u) | LPSPI_TCR_CPHA(1u) | LPSPI_TCR_PRESCALE(0u);
}

void NAFE_HAL_configSpiInterrupt(NAFE_HAL_hdl_t *halHdl, uint32_t interrupts)
{
    if (interrupts & kNafeHalInterruptType_frameComplete)
    {
        halHdl->spiInst->IER |= LPSPI_IER_FCIE_MASK;
    }
    else
    {
        halHdl->spiInst->IER &= ~LPSPI_IER_FCIE_MASK;
    }

    if (interrupts & kNafeHalInterruptType_receiveData)
    {
        halHdl->spiInst->IER |= LPSPI_IER_RDIE_MASK;
    }
    else
    {
        halHdl->spiInst->IER &= ~LPSPI_IER_RDIE_MASK;
    }
}

static void NAFE_HAL_initSpiCsPin(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->csGpioBase->DR |= 1u << halHdl->csGpioIndex;
    halHdl->csGpioBase->GDIR |= 1u << halHdl->csGpioIndex; /* IO Direction: Output */
}

void NAFE_HAL_driveSpiCsPin(NAFE_HAL_hdl_t *halHdl, NAFE_HAL_spiCsLevel_t level)
{
    if (level == kNafeHalSpiCs_high)
    {
        halHdl->csGpioBase->DR |= 1u << halHdl->csGpioIndex;
    }
    else
    {
        halHdl->csGpioBase->DR &= ~(1u << halHdl->csGpioIndex);
    }
}

static void NAFE_HAL_initReadyPin(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->readyPinGpioBase->GDIR &= ~(1u << halHdl->readyPinGpioIndex); /* IO Direction: Input */

    if (halHdl->readyPinGpioIndex < 16u)
    {
        halHdl->readyPinGpioBase->ICR1 &= ~(3u << halHdl->readyPinGpioIndex * 2u);
        halHdl->readyPinGpioBase->ICR1 |= (2u << halHdl->readyPinGpioIndex * 2u); /* Rising edge. */
    }
    else
    {
        halHdl->readyPinGpioBase->ICR2 &= ~(3u << (halHdl->readyPinGpioIndex * 2u - 32u));
        halHdl->readyPinGpioBase->ICR2 |= (2u << (halHdl->readyPinGpioIndex * 2u - 32u)); /* Rising edge. */
    }

    halHdl->readyPinGpioBase->ISR |= 1u << halHdl->readyPinGpioIndex;
}

void NAFE_HAL_configReadyPinInterrupt(NAFE_HAL_hdl_t *halHdl, bool enable)
{
    if (enable)
    {
        halHdl->readyPinGpioBase->IMR |= 1u << halHdl->readyPinGpioIndex;
    }
    else
    {
        halHdl->readyPinGpioBase->IMR &= ~(1u << halHdl->readyPinGpioIndex);
    }
}

static void NAFE_HAL_initSyncPin(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->syncGpioBase->GDIR |= 1u << halHdl->syncGpioIndex; /* IO Direction: Output */
    halHdl->syncGpioBase->DR &= ~(1u << halHdl->syncGpioIndex);
}

void NAFE_HAL_driveSyncPulse(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->syncGpioBase->DR |= 1u << halHdl->syncGpioIndex;
    NAFE_HAL_delay(1u);
    halHdl->syncGpioBase->DR &= ~(1u << halHdl->syncGpioIndex);
}

void NAFE_HAL_writeRegNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                               uint32_t data, uint32_t dataBits)
{
    NAFE_HAL_clearSpiStatus(halHdl);

    /* Set frame size to 16. */
    halHdl->spiInst->TCR = halHdl->spiTcrBaseline \
                         | LPSPI_TCR_CONT(1u) | LPSPI_TCR_RXMSK_MASK \
                         | LPSPI_TCR_FRAMESZ(15u);

    NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_low);

    /* Transfer the 16-bit command. */
    halHdl->spiInst->TDR = cmd;

    /* Set frame size to register size. */
    halHdl->spiInst->TCR = halHdl->spiTcrBaseline \
                         | LPSPI_TCR_CONTC(1u) | LPSPI_TCR_RXMSK_MASK \
                         | LPSPI_TCR_FRAMESZ(dataBits - 1u);

    /* Transfer the data. */
    halHdl->spiInst->TDR = data;
}

void NAFE_HAL_writeRegBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                            uint32_t data, uint32_t dataBits)
{
    NAFE_HAL_writeRegNonBlock(halHdl, cmd, data, dataBits);

    /* Wait for the frame transfer to be completed.*/
    while (!NAFE_HAL_checkSpiStatus(halHdl, kNafeHalInterruptType_frameComplete))
    {
    }

    NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_high);
}

void NAFE_HAL_readRegNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                              uint32_t dataBits)
{
    NAFE_HAL_clearSpiStatus(halHdl);

    /* Set frame size to 16. */
    halHdl->spiInst->TCR = halHdl->spiTcrBaseline \
                         | LPSPI_TCR_CONT(1u) | LPSPI_TCR_RXMSK_MASK \
                         | LPSPI_TCR_FRAMESZ(15u);

    NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_low);

    /* Transfer the 16-bit command. */
    halHdl->spiInst->TDR = cmd;

    /* Set frame size and start the reading transfer. */
    halHdl->spiInst->TCR = halHdl->spiTcrBaseline \
                         | LPSPI_TCR_CONTC(1u) | LPSPI_TCR_TXMSK_MASK \
                         | LPSPI_TCR_FRAMESZ(dataBits - 1u);
}

void NAFE_HAL_readRegBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                           uint32_t *data, uint32_t dataBits)
{
    NAFE_HAL_readRegNonBlock(halHdl, cmd, dataBits);

    /* Wait for the transfer to be completed.*/
    while (!NAFE_HAL_checkSpiStatus(halHdl, kNafeHalInterruptType_receiveData))
    {
    }

    NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_high);

    NAFE_HAL_readSpiRxd(halHdl, data, dataBits, 1u);
}

void NAFE_HAL_sendCmdNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd)
{
    NAFE_HAL_clearSpiStatus(halHdl);

    uint32_t tcr_reg = halHdl->spiTcrBaseline \
                     | LPSPI_TCR_RXMSK_MASK \
                     | LPSPI_TCR_FRAMESZ(15u);

    halHdl->spiInst->TCR = tcr_reg;

    NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_low);

    /* Transfer the 16-bit command. */
    halHdl->spiInst->TDR = cmd;
}

void NAFE_HAL_sendCmdBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                           NAFE_HAL_spiCsLevel_t csEndLevel)
{
    NAFE_HAL_sendCmdNonBlock(halHdl, cmd);

    /* Wait for the transfer to be completed.*/
    while (!NAFE_HAL_checkSpiStatus(halHdl, kNafeHalInterruptType_frameComplete))
    {
    }

    NAFE_HAL_driveSpiCsPin(halHdl, csEndLevel);
}

void NAFE_HAL_receiveResultNonBlock(NAFE_HAL_hdl_t *halHdl, uint32_t dataBits)
{
    /* Start the RX transfer. */
    halHdl->spiInst->TCR = halHdl->spiTcrBaseline \
                         | LPSPI_TCR_TXMSK_MASK \
                         | LPSPI_TCR_FRAMESZ(dataBits - 1u);
}

void NAFE_HAL_receiveResultBlock(NAFE_HAL_hdl_t *halHdl, void *data, uint32_t dataBits,
                                 uint32_t len, bool terminateFrame)
{
    NAFE_HAL_receiveResultNonBlock(halHdl, dataBits);

    /* Wait for the transfer to be completed.*/
    while (!NAFE_HAL_checkSpiStatus(halHdl, kNafeHalInterruptType_receiveData))
    {
    }

    if (terminateFrame)
    {
        NAFE_HAL_driveSpiCsPin(halHdl, kNafeHalSpiCs_high);
    }

    NAFE_HAL_readSpiRxd(halHdl, data, dataBits, len);
}

static inline void NAFE_HAL_clearFlexIOStatus(NAFE_HAL_hdl_t *halHdl)
{
    volatile uint32_t tmp = halHdl->flexioInst->SHIFTBUF[halHdl->flexioShifterIndex];
}

/* Configure FlexIO to use ready pin IO input to trigger DMA requests. */
static void NAFE_HAL_initFlexIO(NAFE_HAL_hdl_t *halHdl)
{
    flexio_shifter_config_t shifterConfig;
    flexio_timer_config_t timerConfig;

    shifterConfig.timerSelect   = halHdl->flexioTimerIndex;
    shifterConfig.pinConfig     = kFLEXIO_PinConfigOutputDisabled;
    shifterConfig.shifterMode   = kFLEXIO_ShifterModeReceive;
    shifterConfig.shifterStop   = kFLEXIO_ShifterStopBitDisable;
    shifterConfig.shifterStart  = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
    shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;

    FLEXIO_SetShifterConfig(halHdl->flexioInst, halHdl->flexioShifterIndex, &shifterConfig);

    timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_PININPUT(halHdl->flexioPinIndex);
    timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
    timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    timerConfig.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
    timerConfig.pinSelect       = halHdl->flexioPinIndex;
    timerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
    timerConfig.timerMode       = kFLEXIO_TimerModeSingle16Bit;
    timerConfig.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
    timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnPinInputShiftPinInput;
    timerConfig.timerReset      = kFLEXIO_TimerResetOnTimerTriggerRisingEdge;
    timerConfig.timerEnable     = kFLEXIO_TimerEnableOnTriggerRisingEdge;
    timerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;
    timerConfig.timerDisable    = kFLEXIO_TimerDisableOnTriggerFallingEdge;
    timerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    timerConfig.timerCompare    = 0xFFFF;

    FLEXIO_SetTimerConfig(halHdl->flexioInst, halHdl->flexioTimerIndex, &timerConfig);

    /* Enable FlexIO. */
    halHdl->flexioInst->CTRL = FLEXIO_CTRL_DBGE_MASK | FLEXIO_CTRL_FLEXEN_MASK;
}

static void NAFE_HAL_initDmaChannels(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl)
{
    static uint32_t flexioRxData;
    static uint32_t lpspiTcrForRxResult;
    uint32_t dmaTransferSizeCode, adcResultBytes;

    if(xferHdl->adcResolutionBits == 24u)
    {
        dmaTransferSizeCode = 2u;
        adcResultBytes = 4u;
    }
    else if(xferHdl->adcResolutionBits == 16u)
    {
        dmaTransferSizeCode = 1u;
        adcResultBytes = 2u;
    }

    lpspiTcrForRxResult = halHdl->spiTcrBaseline | LPSPI_TCR_TXMSK_MASK | LPSPI_TCR_FRAMESZ(xferHdl->adcResolutionBits - 1u);

    halHdl->dmaInst->ERQ = 0U;
    halHdl->dmaInst->INT = 0xFFFFFFFFU;
    halHdl->dmaInst->ERR = 0xFFFFFFFFU;
    halHdl->dmaInst->CR = DMA_CR_GRP1PRI(1u) | DMA_CR_HOE_MASK | DMA_CR_EDBG_MASK | DMA_CR_EMLM(1u);

    /* Configure dmaChnRdyTrig. dmaChnRdyTrig is triggered by FlexIO/Ready Pin, and links dmaChnWrSpi. */
    halHdl->dmamuxInst->CHCFG[halHdl->dmaChnRdyTrig]            = DMAMUX_CHCFG_SOURCE(halHdl->flexioRxDmaReqSrc);
    halHdl->dmamuxInst->CHCFG[halHdl->dmaChnRdyTrig]           |= DMAMUX_CHCFG_ENBL_MASK;

    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].SADDR           = (uint32_t)(halHdl->flexioInst->SHIFTBUF \
                                                                 + halHdl->flexioShifterIndex);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].SOFF            = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].ATTR            = DMA_ATTR_SMOD(0u) | DMA_ATTR_SSIZE(2u) \
                                                                | DMA_ATTR_DMOD(0u) | DMA_ATTR_DSIZE(2u);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].NBYTES_MLOFFNO  = DMA_NBYTES_MLOFFNO_NBYTES(4u);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].SLAST           = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].DADDR           = (uint32_t)&flexioRxData;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].DOFF            = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].DLAST_SGA       = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].CSR             = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].CSR             = DMA_CSR_MAJORELINK_MASK \
                                                                | DMA_CSR_MAJORLINKCH(halHdl->dmaChnWrSpi) \
                                                                | DMA_CSR_DREQ_MASK;

    /* Configure dmaChnWrSpi, used to receive conversion result via LPSPI. */
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].SADDR             = (uint32_t)&lpspiTcrForRxResult;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].SOFF              = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].ATTR              = DMA_ATTR_SMOD(0u) | DMA_ATTR_SSIZE(2u) \
                                                                | DMA_ATTR_DMOD(0u) | DMA_ATTR_DSIZE(2u);
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].NBYTES_MLOFFNO    = DMA_NBYTES_MLOFFNO_NBYTES(xferHdl->chnAmt * 4u);
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].SLAST             = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].DADDR             = (uint32_t)&(halHdl->spiInst->TCR);
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].DOFF              = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].DLAST_SGA         = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].CSR               = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].CSR               = DMA_CSR_DREQ_MASK;

    /* Configure dmaChnRdSpi, used to read LPSPI Rx FIFO. */
    halHdl->dmamuxInst->CHCFG[halHdl->dmaChnRdSpi]              = DMAMUX_CHCFG_SOURCE(halHdl->spiRxDmaReqSrc);
    halHdl->dmamuxInst->CHCFG[halHdl->dmaChnRdSpi]             |= DMAMUX_CHCFG_ENBL_MASK;

    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].SADDR             = (uint32_t)&(halHdl->spiInst->RDR);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].SOFF              = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].ATTR              = DMA_ATTR_SMOD(0u) | DMA_ATTR_SSIZE(dmaTransferSizeCode) \
                                                                | DMA_ATTR_DMOD(0u) | DMA_ATTR_DSIZE(dmaTransferSizeCode);
    if (xferHdl->chnAmt == 1u)
    {
        halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].NBYTES_MLNO   = adcResultBytes;
    }
    else
    {
        halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].NBYTES_MLOFFYES = DMA_NBYTES_MLOFFYES_DMLOE_MASK \
                            | DMA_NBYTES_MLOFFYES_MLOFF(- xferHdl->contSampleAmt * xferHdl->chnAmt * adcResultBytes + adcResultBytes)
                            | DMA_NBYTES_MLOFFYES_NBYTES(xferHdl->chnAmt * adcResultBytes);
    }
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].SLAST             = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].DADDR             = (uint32_t)(xferHdl->pResult);
    if (xferHdl->chnAmt == 1u)
    {
        halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].DOFF          = adcResultBytes;
    }
    else
    {
        halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].DOFF          = xferHdl->contSampleAmt * adcResultBytes;
    }
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].DLAST_SGA         = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].CSR               = 0u;
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].CSR               = DMA_CSR_DREQ_MASK | DMA_CSR_INTMAJOR_MASK;
}

#define MAX_MAJOR_LOOP_SIZE 255

static void NAFE_HAL_refreshDmaTcd(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl, bool updateRdSpiDaddr)
{
    static uint32_t dmaMajorLoopSize;
    uint32_t remainSamples;

    remainSamples = xferHdl->contSampleAmt - xferHdl->contSampleCnt;
    dmaMajorLoopSize = remainSamples <= MAX_MAJOR_LOOP_SIZE ? remainSamples : MAX_MAJOR_LOOP_SIZE;
    xferHdl->contSampleCnt += dmaMajorLoopSize;

    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].CITER_ELINKYES  = DMA_CITER_ELINKYES_ELINK(1u) \
                                                                | DMA_CITER_ELINKYES_LINKCH(halHdl->dmaChnWrSpi) \
                                                                | DMA_CITER_ELINKYES_CITER(dmaMajorLoopSize);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdyTrig].BITER_ELINKYES  = DMA_BITER_ELINKYES_ELINK(1u) \
                                                                | DMA_BITER_ELINKYES_LINKCH(halHdl->dmaChnWrSpi) \
                                                                | DMA_BITER_ELINKYES_BITER(dmaMajorLoopSize);
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].CITER_ELINKNO     = DMA_CITER_ELINKNO_ELINK(0u) \
                                                                | DMA_CITER_ELINKNO_CITER(dmaMajorLoopSize);
    halHdl->dmaInst->TCD[halHdl->dmaChnWrSpi].BITER_ELINKNO     = DMA_BITER_ELINKNO_ELINK(0u) \
                                                                | DMA_BITER_ELINKNO_BITER(dmaMajorLoopSize);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].CITER_ELINKNO     = DMA_CITER_ELINKNO_ELINK(0u) \
                                                                | DMA_CITER_ELINKNO_CITER(dmaMajorLoopSize);
    halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].BITER_ELINKNO     = DMA_BITER_ELINKNO_ELINK(0u) \
                                                                | DMA_BITER_ELINKNO_BITER(dmaMajorLoopSize);

    if(updateRdSpiDaddr)
    {
        halHdl->dmaInst->TCD[halHdl->dmaChnRdSpi].DADDR = (uint32_t)(xferHdl->pResult);
    }

    /* Enable DMA requests. */
    halHdl->dmaInst->SERQ = DMA_SERQ_SERQ(halHdl->dmaChnRdSpi);
    halHdl->dmaInst->SERQ = DMA_SERQ_SERQ(halHdl->dmaChnWrSpi);
    halHdl->dmaInst->SERQ = DMA_SERQ_SERQ(halHdl->dmaChnRdyTrig);
}

void NAFE_HAL_prepareDmaXfer(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl)
{

    NAFE_HAL_refreshDmaTcd(halHdl, xferHdl, true);

    NAFE_HAL_clearSpiStatus(halHdl);
    halHdl->spiInst->DER |= LPSPI_DER_RDDE_MASK;    /* Enable SPI RXD DMA. */

    NAFE_HAL_clearFlexIOStatus(halHdl);
    FLEXIO_EnableShifterStatusDMA(halHdl->flexioInst, 1u<<halHdl->flexioShifterIndex, true); /* Enable FlexIO DMA. */
}

void NAFE_HAL_dmaIrqHandle(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl)
{
    if (xferHdl->contSampleCnt < xferHdl->contSampleAmt)
    {
        NAFE_HAL_refreshDmaTcd(halHdl, xferHdl, false);
    }
    else
    {
        FLEXIO_EnableShifterStatusDMA(halHdl->flexioInst, 1u<<halHdl->flexioShifterIndex, false); /* Disable FlexIO DMA. */
        halHdl->spiInst->DER &= ~LPSPI_DER_RDDE_MASK;    /* Disable SPI RXD DMA. */

        xferHdl->nonBlockXferState = kNafeNonBlockXferState_done;
    }

    /* Clear flags. */
    halHdl->dmaInst->CDNE |= DMA_CDNE_CDNE(halHdl->dmaChnRdyTrig);
    halHdl->dmaInst->CDNE |= DMA_CDNE_CDNE(halHdl->dmaChnWrSpi);
    halHdl->dmaInst->CDNE |= DMA_CDNE_CDNE(halHdl->dmaChnRdSpi);

    halHdl->dmaInst->CINT |= DMA_CINT_CINT(halHdl->dmaChnRdSpi);

    __DSB();
}
