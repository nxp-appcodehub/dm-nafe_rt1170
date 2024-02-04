/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _NAFE_HAL_H_
#define _NAFE_HAL_H_

#include "fsl_device_registers.h"
#include "board.h"
#include "nafe13388.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef enum
{
    kNafeHalSpiCs_low = 0u,
    kNafeHalSpiCs_high
} NAFE_HAL_spiCsLevel_t;

typedef enum
{
    kNafeHalInterruptType_frameComplete = 1u << 0u,
    kNafeHalInterruptType_receiveData = 1u << 1u,
} NAFE_HAL_spiStatus_t;

typedef struct
{
    LPSPI_Type *spiInst;
    uint32_t spiTcrBaseline;
    GPIO_Type *csGpioBase;
    uint32_t csGpioIndex;
    GPIO_Type *readyPinGpioBase;
    uint32_t readyPinGpioIndex;
    GPIO_Type *syncGpioBase;
    uint32_t syncGpioIndex;

    /* For DMA approaches. */
    FLEXIO_Type *flexioInst;
    DMA_Type *dmaInst;
    DMAMUX_Type *dmamuxInst;
    uint32_t dmaChnRdyTrig;
    uint32_t dmaChnWrSpi;
    uint32_t dmaChnRdSpi;
    uint32_t flexioTimerIndex;
    uint32_t flexioShifterIndex;
    uint32_t flexioPinIndex;
    uint32_t flexioRxDmaReqSrc;
    uint32_t spiRxDmaReqSrc;
} NAFE_HAL_hdl_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
void NAFE_HAL_init(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl);

static void inline NAFE_HAL_delay(uint32_t us)
{
    SDK_DelayAtLeastUs(us, BOARD_BOOTCLOCKRUN_CORE_CLOCK);
}

static inline bool NAFE_HAL_checkSpiStatus(NAFE_HAL_hdl_t *halHdl, uint32_t status)
{
    bool rel = false;

    if (status & kNafeHalInterruptType_frameComplete)
    {
        rel = halHdl->spiInst->SR &= LPSPI_SR_FCF_MASK;
    }
    else if (status & kNafeHalInterruptType_receiveData)
    {
        rel = halHdl->spiInst->SR &= LPSPI_SR_RDF_MASK;
    }

    return rel;
}

static inline void NAFE_HAL_clearSpiStatus(NAFE_HAL_hdl_t *halHdl)
{
    /* Reset RX FIFO and TX FIFO. */
    halHdl->spiInst->CR |= LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;

    /* Clear Status Register. */
    halHdl->spiInst->SR |= LPSPI_SR_FCF_MASK;
}

static inline void NAFE_HAL_configSpiRxFifoTrigLevel(NAFE_HAL_hdl_t *halHdl,
                                                     uint32_t rxFifoTrigLevel)
{
    halHdl->spiInst->FCR = LPSPI_FCR_RXWATER(rxFifoTrigLevel - 1u);
}

void NAFE_HAL_configSpiInterrupt(NAFE_HAL_hdl_t *halHdl, uint32_t interrupts);

static inline void NAFE_HAL_readSpiRxd(NAFE_HAL_hdl_t *halHdl, void *data,
                                       uint32_t dataBits, uint32_t len)
{
    if(dataBits == 24u)
    {
        uint32_t * u32Data = (uint32_t *)data;

        while (len--)
        {
            *u32Data++ = halHdl->spiInst->RDR;
        }
    }
    else if(dataBits == 16u)
    {
        uint16_t * u16Data = (uint16_t *)data;

        while (len--)
        {
            *u16Data++ = halHdl->spiInst->RDR;
        }
    }
}

void NAFE_HAL_driveSpiCsPin(NAFE_HAL_hdl_t *halHdl, NAFE_HAL_spiCsLevel_t level);

void NAFE_HAL_configReadyPinInterrupt(NAFE_HAL_hdl_t *halHdl, bool enable);

static inline void NAFE_HAL_clearReadyPinStatus(NAFE_HAL_hdl_t *halHdl)
{
    halHdl->readyPinGpioBase->ISR |= 1u << halHdl->readyPinGpioIndex;
}

static inline bool NAFE_HAL_checkReadyPin(NAFE_HAL_hdl_t *halHdl)
{
    return (halHdl->readyPinGpioBase->ISR & (1u << halHdl->readyPinGpioIndex)) ? \
            true : false;
}

void NAFE_HAL_driveSyncPulse(NAFE_HAL_hdl_t *halHdl);

void NAFE_HAL_writeRegNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                               uint32_t data, uint32_t dataBits);

void NAFE_HAL_writeRegBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                            uint32_t data, uint32_t dataBits);

void NAFE_HAL_readRegNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                              uint32_t dataBits);

void NAFE_HAL_readRegBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                           uint32_t *data, uint32_t dataBits);

void NAFE_HAL_sendCmdNonBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd);

void NAFE_HAL_sendCmdBlock(NAFE_HAL_hdl_t *halHdl, uint16_t cmd,
                           NAFE_HAL_spiCsLevel_t csEndLevel);

void NAFE_HAL_receiveResultNonBlock(NAFE_HAL_hdl_t *halHdl, uint32_t dataBits);

void NAFE_HAL_receiveResultBlock(NAFE_HAL_hdl_t *halHdl, void *data, uint32_t dataBits,
                                 uint32_t len, bool terminateFrame);

void NAFE_HAL_prepareDmaXfer(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl);

void NAFE_HAL_dmaIrqHandle(NAFE_HAL_hdl_t *halHdl, NAFE_xferHdl_t *xferHdl);

#endif
