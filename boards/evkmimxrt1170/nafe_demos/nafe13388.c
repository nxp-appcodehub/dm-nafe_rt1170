/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nafe13388.h"
#include "nafe_hal.h"

/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/
static void NAFE_resetDevice(NAFE_devHdl_t *devHdl);

static void NAFE_combCmdForAccessRegs(uint16_t devAddr, uint16_t regAddr,
                            NAFE_regAccessType_t accessType, uint16_t *combCmd);

static void NAFE_combCmdForInstructionCmds(uint16_t devAddr, uint16_t regAddr,
                            uint16_t *combCmd);

static inline void NAFE_writeRegBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                      uint32_t data, NAFE_regDataSize_t regSize);

static inline void NAFE_readRegNonBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                        NAFE_regDataSize_t regSize);

static inline void NAFE_readRegBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                     uint32_t *data, NAFE_regDataSize_t regSize);

static inline void NAFE_sendCmdNonBlock(NAFE_devHdl_t *devHdl, uint32_t cmd);

static inline void NAFE_sendCmdBlock(NAFE_devHdl_t *devHdl, uint32_t cmd,
                                     uint32_t csEndLevel);

static void NAFE_triggerSample(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_adcCode2Voltage(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_scsrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_sccrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_mcmrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_mccrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

static void NAFE_readResult(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl);

/*******************************************************************************
 * Static Varibles.
 ******************************************************************************/
/* Initialization gains based on PGA setting for back calculation the results to volt units */
static float g_adcCode2VoltageFactor[8]=
{
    10.0 / 0.2 / 0x1000000,  /* input_range / PGA_gain_0.2  / 2^24) */
    10.0 / 0.4 / 0x1000000,  /* input_range / PGA_gain_0.4  / 2^24) */
    10.0 / 0.8 / 0x1000000,  /* input_range / PGA_gain_0.8  / 2^24) */
    10.0 / 1.0 / 0x1000000,  /* input_range / PGA_gain_1.0  / 2^24) */
    10.0 / 2.0 / 0x1000000,  /* input_range / PGA_gain_2.0  / 2^24) */
    10.0 / 4.0 / 0x1000000,  /* input_range / PGA_gain_4.0  / 2^24) */
    10.0 / 8.0 / 0x1000000,  /* input_range / PGA_gain_8.0  / 2^24) */
    10.0 / 16.0/ 0x1000000,  /* input_range / PGA_gain_16.0 / 2^24) */
};

/*******************************************************************************
 * Functions
 ******************************************************************************/
/* Initialize the device. */
int32_t NAFE_init(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    uint32_t u32Tmp;

    if (devHdl->sysConfig->adcResolutionCode == kNafeAdcResolution_24bits)
    {
        xferHdl->adcResolutionBits = kNafeRegDataSize_24bits;
    }
    else if (devHdl->sysConfig->adcResolutionCode == kNafeAdcResolution_16bits)
    {
        xferHdl->adcResolutionBits = kNafeRegDataSize_16bits;
    }
    NAFE_HAL_init(devHdl->halHdl, xferHdl);

    /* Reset the device. */
    NAFE_resetDevice(devHdl);

    /* Configure channel registers. */
    for (int32_t i = 0; i < xferHdl->chnAmt; i++)
    {
        NAFE_sendCmdBlock(devHdl, NAFE_CMD_CH0 + devHdl->chConfig[i].chnIndex, 1u);
        NAFE_writeRegBlock(devHdl, NAFE_REG_CH_CONFIG0, \
                          (devHdl->chConfig[i].hvAip << 12u) \
                        | (devHdl->chConfig[i].hvAin << 8u) \
                        | (devHdl->chConfig[i].gain << 5u) \
                        | (devHdl->chConfig[i].inputSel << 4u) \
                        | (devHdl->chConfig[i].lv << 1u) \
                        | (devHdl->chConfig[i].tcc), \
                            kNafeRegDataSize_16bits);
        NAFE_writeRegBlock(devHdl, NAFE_REG_CH_CONFIG1, \
                           devHdl->chConfig[i].dataRateCode << 3u \
                        |  devHdl->chConfig[i].adcSinc, \
                           kNafeRegDataSize_16bits);
        NAFE_writeRegBlock(devHdl, NAFE_REG_CH_CONFIG2, \
                           devHdl->chConfig[i].chDelayCode << 10u \
                        |  devHdl->chConfig[i].adcSettling << 9u \
                        |  devHdl->chConfig[i].filterReset << 8u \
                        |  devHdl->chConfig[i].chChop << 7u, \
                           kNafeRegDataSize_16bits);
        NAFE_writeRegBlock(devHdl, NAFE_REG_CH_CONFIG3, \
                           devHdl->chConfig[i].viexVi << 15u \
                        |  devHdl->chConfig[i].viexPol << 14u \
                        |  devHdl->chConfig[i].viexMag << 10u \
                        |  devHdl->chConfig[i].viexChop << 6u \
                        |  devHdl->chConfig[i].viexAipEn << 3u \
                        |  devHdl->chConfig[i].viexAinEn,
                           kNafeRegDataSize_16bits);
    }

    /* Config sys reg. */
    uint32_t enabledChnMask = 0;
    for (uint32_t i = 0; i < xferHdl->chnAmt; i++)
    {
        enabledChnMask |= 1u << devHdl->chConfig[i].chnIndex;
    }

    NAFE_readRegBlock(devHdl, NAFE_REG_SYS_CONFIG0, &u32Tmp, kNafeRegDataSize_16bits);
    u32Tmp &= ~(1u << 4u);
    u32Tmp |= devHdl->sysConfig->readyPinSeqMode << 4u;
    u32Tmp &= ~(1u << 5u);
    u32Tmp |= devHdl->sysConfig->triggerMode << 5u;
    u32Tmp &= ~(1u << 14u);
    u32Tmp |= devHdl->sysConfig->adcResolutionCode << 14;
    NAFE_writeRegBlock(devHdl, NAFE_REG_SYS_CONFIG0, u32Tmp, kNafeRegDataSize_16bits);
    NAFE_writeRegBlock(devHdl, NAFE_REG_CH_CONFIG4, \
                       enabledChnMask, kNafeRegDataSize_16bits);

    NAFE_HAL_delay(1u);

    return 0;
}

void NAFE_startSample(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    uint32_t interrupts = 0u;
    uint32_t rxFifoTrigLevel = 1u;
    bool readyInterrupt = false;

    NAFE_HAL_clearSpiStatus(devHdl->halHdl);
    NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);

    xferHdl->contSampleCnt= 0u;                                    /* For continuous sample modes. */
    xferHdl->chnSampleCnt = 0u;                                    /* For multi-channel sample modes. */
    xferHdl->nonBlockXferState = kNafeNonBlockXferState_initial;   /* For non-block sample modes. */

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
        case kNafeSampleMode_sccrBlock:
            assert(xferHdl->chnAmt == 1u);
            break;
        case kNafeSampleMode_mcmrBlock:
        case kNafeSampleMode_mccrBlock:
            assert(xferHdl->chnAmt <= NAFE_CHN_AMT_MAX);
            break;
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_sccrNonBlock:
            assert(xferHdl->chnAmt == 1u);
            interrupts = kNafeHalInterruptType_frameComplete;
            readyInterrupt = true;
            break;
        case kNafeSampleMode_mcmrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
            assert(xferHdl->chnAmt <= NAFE_CHN_AMT_MAX);
            interrupts = kNafeHalInterruptType_frameComplete;
            readyInterrupt = true;
            break;
        case kNafeSampleMode_sccrDma:
            assert(xferHdl->chnAmt == 1u);
            NAFE_HAL_prepareDmaXfer(devHdl->halHdl, xferHdl);
            break;
        case kNafeSampleMode_mccrDma:
            assert(xferHdl->chnAmt <= NAFE_CHN_AMT_MAX);
            NAFE_HAL_prepareDmaXfer(devHdl->halHdl, xferHdl);
            rxFifoTrigLevel = xferHdl->chnAmt;
            break;
        default:
            break;
    }

    NAFE_HAL_configSpiRxFifoTrigLevel(devHdl->halHdl, rxFifoTrigLevel);
    NAFE_HAL_configSpiInterrupt(devHdl->halHdl, interrupts);
    NAFE_HAL_configReadyPinInterrupt(devHdl->halHdl, readyInterrupt);

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
            NAFE_scsrBlock(devHdl, xferHdl);
            break;
        case kNafeSampleMode_sccrBlock:
            NAFE_sccrBlock(devHdl, xferHdl);
            break;
        case kNafeSampleMode_mcmrBlock:
            NAFE_mcmrBlock(devHdl, xferHdl);
            break;
        case kNafeSampleMode_mccrBlock:
            NAFE_mccrBlock(devHdl, xferHdl);
            break;
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            NAFE_triggerSample(devHdl, xferHdl);
            break;
        default:
            break;
    }
}

void NAFE_terminateSample(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
        case kNafeSampleMode_mcmrBlock:
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
            NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
            break;
        case kNafeSampleMode_sccrBlock:
        case kNafeSampleMode_mccrBlock:
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            NAFE_sendCmdBlock(devHdl, NAFE_CMD_ABORT, 1u);
            NAFE_HAL_delay(2u);
            break;
        default:
            break;
    }

    NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);

    devHdl->currentSampleMode = kNafeSampleMode_none;
}

static void NAFE_resetDevice(NAFE_devHdl_t *devHdl)
{
    NAFE_sendCmdBlock(devHdl, NAFE_CMD_RESET, 1u);
    NAFE_HAL_delay(50u * 1000u);
}

/* Generate combined command for accessing registers, including device address, W/R access type,
   and register address.*/
static void NAFE_combCmdForAccessRegs(uint16_t devAddr, uint16_t regAddr,
                            NAFE_regAccessType_t accessType, uint16_t *combCmd)
{
    assert(devAddr == 0u || devAddr == 1u);

    *combCmd = devAddr;
    *combCmd <<= 15u;               /*bit15: hardware address */
    *combCmd |= accessType << 14u;  /*bit14: W = 0, R = 1 */
    *combCmd |= regAddr << 1u;      /*bit1~bit13: register address */
}

/* Generate combined command for instruction commands, including device address and instruction commands.*/
static void NAFE_combCmdForInstructionCmds(uint16_t devAddr, uint16_t cmd,
                            uint16_t *combCmd)
{
    assert(devAddr == 0u || devAddr == 1u);

    *combCmd = devAddr;
    *combCmd <<= 15u;               /*bit15: hardware address */
    *combCmd |= cmd << 1u;          /*bit1~bit14: instruction command */
}

static inline void NAFE_writeRegBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                      uint32_t data, NAFE_regDataSize_t regSize)
{
    uint16_t combCmd;

    NAFE_combCmdForAccessRegs(devHdl->devAddr, regAddr, kNafeRegAccess_write, &combCmd);
    NAFE_HAL_writeRegBlock(devHdl->halHdl, combCmd, data, regSize);
}

static inline void NAFE_readRegNonBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                        NAFE_regDataSize_t regSize)
{
    uint16_t combCmd;

    NAFE_combCmdForAccessRegs(devHdl->devAddr, regAddr, kNafeRegAccess_read, &combCmd);
    NAFE_HAL_readRegNonBlock(devHdl->halHdl, combCmd, regSize);
}

static inline void NAFE_readRegBlock(NAFE_devHdl_t *devHdl, uint16_t regAddr,
                                     uint32_t *data, NAFE_regDataSize_t regSize)
{
    uint16_t combCmd;

    NAFE_combCmdForAccessRegs(devHdl->devAddr, regAddr, kNafeRegAccess_read, &combCmd);
    NAFE_HAL_readRegBlock(devHdl->halHdl, combCmd, data, regSize);
}

static inline void NAFE_sendCmdNonBlock(NAFE_devHdl_t *devHdl, uint32_t cmd)
{
    uint16_t combCmd;

    NAFE_combCmdForInstructionCmds(devHdl->devAddr, cmd, &combCmd);
    NAFE_HAL_sendCmdNonBlock(devHdl->halHdl, combCmd);
}

static inline void NAFE_sendCmdBlock(NAFE_devHdl_t *devHdl, uint32_t cmd,
                                     uint32_t csEndLevel)
{
    uint16_t combCmd;

    NAFE_combCmdForInstructionCmds(devHdl->devAddr, cmd, &combCmd);
    NAFE_HAL_sendCmdBlock(devHdl->halHdl, combCmd, (NAFE_HAL_spiCsLevel_t)csEndLevel);
}

/* Trigger to start the sample and conversion. */
static void NAFE_triggerSample(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    uint16_t cmd;

    switch (xferHdl->sampleMode)
    {
        case kNafeSampleMode_scsrBlock:
        case kNafeSampleMode_sccrBlock:
        case kNafeSampleMode_mcmrBlock:
        case kNafeSampleMode_mccrBlock:
        case kNafeSampleMode_sccrDma:
        case kNafeSampleMode_mccrDma:
            if (devHdl->currentSampleMode != xferHdl->sampleMode \
            || devHdl->sysConfig->triggerMode != kNafeTrigger_syncPin)
            {
                if (xferHdl->sampleMode == kNafeSampleMode_sccrDma)
                {
                    cmd = NAFE_CMD_SC;
                }
                else if (xferHdl->sampleMode == kNafeSampleMode_mccrDma)
                {
                    cmd = NAFE_CMD_MC;
                }
                else
                {
                    cmd = xferHdl->sampleMode - kNafeSampleMode_scsrBlock + NAFE_CMD_SS;
                }

                /* Send the command. */
                if (devHdl->readResultFrameMode == kNafeReadResult_multiFrame)
                {
                    NAFE_sendCmdBlock(devHdl, cmd, 1u);
                }
                else
                {
                    if (devHdl->sysConfig->triggerMode == kNafeTrigger_syncPin)
                    {
                        /* Wait cmd frame to be end, then send the sync signal.*/
                        NAFE_sendCmdBlock(devHdl, cmd, 0u);
                    }
                    else
                    {
                        /* No need to wait cmd frame to be end. */
                        NAFE_sendCmdNonBlock(devHdl, cmd);
                    }
                }

                devHdl->currentSampleMode = xferHdl->sampleMode;
            }
            /* Trigger the sample with SYNC signal. */
            if (devHdl->sysConfig->triggerMode == kNafeTrigger_syncPin)
            {
                NAFE_HAL_driveSyncPulse(devHdl->halHdl);
            }
            break;
        case kNafeSampleMode_scsrNonBlock:
        case kNafeSampleMode_sccrNonBlock:
        case kNafeSampleMode_mcmrNonBlock:
        case kNafeSampleMode_mccrNonBlock:
            if (devHdl->currentSampleMode != xferHdl->sampleMode \
            || devHdl->sysConfig->triggerMode == kNafeTrigger_spiCmd)
            {
                cmd = xferHdl->sampleMode - kNafeSampleMode_scsrNonBlock + NAFE_CMD_SS;

                NAFE_sendCmdNonBlock(devHdl, cmd);

                devHdl->currentSampleMode = xferHdl->sampleMode;
                xferHdl->nonBlockXferState = kNafeNonBlockXferState_modeCmdSent;
            }
            else
            {
                xferHdl->nonBlockXferState = kNafeNonBlockXferState_waitForReady;

                NAFE_HAL_driveSyncPulse(devHdl->halHdl);
            }
            break;
        default:
            break;
    }
}

/* Convert result array from 24-bit signed int type to float type. */
static void NAFE_adcCode2Voltage(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    void * pResult = xferHdl->pResult;
    NAFE_regDataSize_t dataSizeInBits = xferHdl->adcResolutionBits;
    uint32_t len;
    float adcCode2VoltageFactor;

    uint32_t *u32Result;
    uint16_t *u16Result;
    float *f32Result;
    int32_t s32Tmp;
    uint32_t u32Tmp;

    for (int32_t i = xferHdl->chnAmt - 1; i >= 0; i--)
    {
        len = xferHdl->contSampleAmt;
        f32Result = (float *)pResult    + len * (i + 1) - 1u;
        u32Result = (uint32_t *)pResult + len * (i + 1) - 1u;
        u16Result = (uint16_t *)pResult + len * (i + 1) - 1u;
        adcCode2VoltageFactor = g_adcCode2VoltageFactor[devHdl->chConfig[i].gain];

        if (dataSizeInBits == kNafeRegDataSize_24bits)
        {
            while (len--)
            {
                *u32Result <<= 8u;
                s32Tmp = (int32_t)*u32Result;
                s32Tmp >>= 8u;
                *f32Result = s32Tmp * adcCode2VoltageFactor;
                u32Result--;
                f32Result--;
            }
        }
        else if (dataSizeInBits == kNafeRegDataSize_16bits)
        {
            while (len--)
            {
                u32Tmp = *u16Result;
                u32Tmp <<= 16u;
                s32Tmp = (int32_t)u32Tmp;
                s32Tmp >>= 8u;
                *f32Result = s32Tmp * adcCode2VoltageFactor;
                u16Result--;
                f32Result--;
            }
        }
    }
}

static void NAFE_scsrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    NAFE_triggerSample(devHdl, xferHdl);

    /* Wait for the conversion to be done. */
    while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
    {
    }
    NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);

    /* Read the result. */
    if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
    {
        bool terminateFrame = (devHdl->sysConfig->triggerMode == kNafeTrigger_spiCmd) ? \
                              true : false;
        NAFE_HAL_receiveResultBlock(devHdl->halHdl, xferHdl->pResult, \
            (uint32_t)xferHdl->adcResolutionBits, 1u, terminateFrame);
    }
    else
    {
        NAFE_readRegBlock(devHdl, NAFE_REG_CH_DATA0 + devHdl->chConfig[0].chnIndex, \
                          xferHdl->pResult, xferHdl->adcResolutionBits);
    }

    NAFE_adcCode2Voltage(devHdl, xferHdl);

    /* Do not call NAFE_terminateSample() or rest devHdl->currentSampleMode,
       So that keeping the sample status in SCSR. And enable the sync pin trigger
       next if required.
    */
}

static void NAFE_sccrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    void *pResult;
    NAFE_triggerSample(devHdl, xferHdl);

    while (xferHdl->contSampleCnt < xferHdl->contSampleAmt)
    {
        /* Wait for the conversion to be done. */
        while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
        {
        }
        NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);

        if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
        {
            pResult = (uint32_t *)xferHdl->pResult + xferHdl->contSampleCnt;
        }
        else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
        {
            pResult = (uint16_t *)xferHdl->pResult + xferHdl->contSampleCnt;
        }

        /* Read the result. */
        if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
        {
            NAFE_HAL_receiveResultBlock(devHdl->halHdl, \
                pResult, (uint32_t)xferHdl->adcResolutionBits, 1u, false);
        }
        else
        {
            NAFE_readRegBlock(devHdl, NAFE_REG_CH_DATA0 + devHdl->chConfig[0].chnIndex, \
                pResult, xferHdl->adcResolutionBits);
        }

        xferHdl->contSampleCnt++;

        if (NULL != xferHdl->blockingCallback)
        {
            xferHdl->blockingCallback();
        }
    }

    if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
    {
        NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
    }
    NAFE_terminateSample(devHdl, xferHdl);

    NAFE_adcCode2Voltage(devHdl, xferHdl);
}

static void NAFE_mcmrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    bool terminateFrame = false;
    void *pResult;

    NAFE_triggerSample(devHdl, xferHdl);

    if (devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onSequencer)
    {
        /* Wait for all conversions to be done. */
        while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
        {
        }
        NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);
    }
    for (uint32_t i = 0u; i < xferHdl->chnAmt; i++)
    {
        if (devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onConversion)
        {
            /* Wait for each conversion to be done. */
            while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
            {
            }
            NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);
        }

        if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
        {
            pResult = (uint32_t *)xferHdl->pResult + i;
        }
        else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
        {
            pResult = (uint16_t *)xferHdl->pResult + i;
        }

        /* Read the result. */
        if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
        {
            if (i == xferHdl->chnAmt - 1u \
            && devHdl->sysConfig->triggerMode == kNafeTrigger_spiCmd)
            {
                terminateFrame = true;
            }
            NAFE_HAL_receiveResultBlock(devHdl->halHdl, \
                            pResult, (uint32_t)xferHdl->adcResolutionBits, 1u, terminateFrame);
        }
        else
        {
            NAFE_readRegBlock(devHdl, NAFE_REG_CH_DATA0 + devHdl->chConfig[i].chnIndex, \
                              pResult, xferHdl->adcResolutionBits);
        }
    }

    NAFE_adcCode2Voltage(devHdl, xferHdl);

    /* Do not call NAFE_terminateSample() or rest devHdl->currentSampleMode,
       So that keeping the sample status in MCMR. And enable the sync pin trigger
       next if required (for Rev.B).
    */
}

static void NAFE_mccrBlock(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    void *pResult;

    NAFE_triggerSample(devHdl, xferHdl);

    while (xferHdl->contSampleCnt < xferHdl->contSampleAmt)
    {
        if (devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onSequencer)
        {
            /* Wait for all conversions to be done. */
            while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
            {
            }
            NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);
        }
        for (uint32_t i = 0u; i < xferHdl->chnAmt; i++)
        {
            if (devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onConversion)
            {
                /* Wait for each conversion to be done. */
                while (!NAFE_HAL_checkReadyPin(devHdl->halHdl))
                {
                }
                NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);
            }

            if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
            {
                pResult = (uint32_t *)xferHdl->pResult + xferHdl->contSampleAmt * i + xferHdl->contSampleCnt;
            }
            else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
            {
                pResult = (uint16_t *)xferHdl->pResult + xferHdl->contSampleAmt * i + xferHdl->contSampleCnt;
            }

            /* Read the result. */
            if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
            {
                NAFE_HAL_receiveResultBlock(devHdl->halHdl, \
                    pResult, (uint32_t)xferHdl->adcResolutionBits, 1u, false);
            }
            else
            {
                NAFE_readRegBlock(devHdl, NAFE_REG_CH_DATA0 + devHdl->chConfig[i].chnIndex, \
                    pResult, xferHdl->adcResolutionBits);
            }
        }
        xferHdl->contSampleCnt++;

        if (NULL != xferHdl->blockingCallback)
        {
            xferHdl->blockingCallback();
        }
    }

    if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
    {
        NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
    }
    NAFE_terminateSample(devHdl, xferHdl);

    NAFE_adcCode2Voltage(devHdl, xferHdl);
}

/* Send SPI Rx clock pattern or read result registers to read the conversion results. */
static void NAFE_readResult(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame)
    {
       if ((xferHdl->sampleMode == kNafeSampleMode_mcmrNonBlock \
         || xferHdl->sampleMode == kNafeSampleMode_mccrNonBlock)
         && devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onSequencer)
        {
            for (uint32_t i = 0u; i < xferHdl->chnAmt; i++)
            {
                NAFE_HAL_receiveResultNonBlock(devHdl->halHdl, (uint32_t)xferHdl->adcResolutionBits);
            }
        }
        else
        {
            NAFE_HAL_receiveResultNonBlock(devHdl->halHdl, (uint32_t)xferHdl->adcResolutionBits);
        }
    }
    else
    {
        uint32_t chConfigIndex;

        if (xferHdl->sampleMode == kNafeSampleMode_scsrNonBlock \
         || xferHdl->sampleMode == kNafeSampleMode_sccrNonBlock)
        {
            chConfigIndex = 0u;
        }
        else
        {
            chConfigIndex = xferHdl->chnSampleCnt;
        }

        NAFE_readRegNonBlock(devHdl,
                NAFE_REG_CH_DATA0 + devHdl->chConfig[chConfigIndex].chnIndex,
                xferHdl->adcResolutionBits);
    }
}

/* Ready pin and SPI IRQ handle. Process the non-block (interrupt) approach
   state machine. */
void NAFE_irqHandle(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl,
                    NAFE_interruptSrc_t interrupt)
{
    void *pResult;
    uint32_t rxFifoTrigLevel;
    uint32_t interrupts;
    bool readResult = false;
    bool roundSampleEnd = false;
    bool contSampleEnd = false;

    switch (xferHdl->nonBlockXferState)
    {
        case kNafeNonBlockXferState_modeCmdSent:
            assert(interrupt == kNafeInterrupt_spi);
            NAFE_HAL_clearSpiStatus(devHdl->halHdl);

            if (devHdl->readResultFrameMode == kNafeReadResult_multiFrame)
            {
                NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
            }
            if (devHdl->sysConfig->triggerMode == kNafeTrigger_syncPin)
            {
                NAFE_triggerSample(devHdl, xferHdl); /* Output the sync pulse. */
            }
            else
            {
                xferHdl->nonBlockXferState = kNafeNonBlockXferState_waitForReady;
            }
            break;
        case kNafeNonBlockXferState_waitForReady:
            assert(interrupt == kNafeInterrupt_readyPin);
            NAFE_HAL_clearReadyPinStatus(devHdl->halHdl);
            xferHdl->nonBlockXferState = kNafeNonBlockXferState_ready;
            /* No "break" statement here. */
        case kNafeNonBlockXferState_ready:
            /* Set Rx FIFO trigger level, and enable Rx FIFO interrupt,
               and disable frame complete interrupt. */
            interrupts = kNafeHalInterruptType_receiveData;

            if ((xferHdl->sampleMode == kNafeSampleMode_mcmrNonBlock \
              || xferHdl->sampleMode == kNafeSampleMode_mccrNonBlock) \
              && devHdl->readResultFrameMode == kNafeReadResult_singleFrame \
              && devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onSequencer)
            {
                rxFifoTrigLevel = xferHdl->chnAmt;
            }
            else
            {
                rxFifoTrigLevel = 1u;
            }
            NAFE_HAL_configSpiRxFifoTrigLevel(devHdl->halHdl, rxFifoTrigLevel);
            NAFE_HAL_configSpiInterrupt(devHdl->halHdl, interrupts);

            NAFE_readResult(devHdl, xferHdl);
            xferHdl->nonBlockXferState = kNafeNonBlockXferState_rxClkSending;
            break;
        case kNafeNonBlockXferState_rxClkSending:
            assert(interrupt == kNafeInterrupt_spi);
            xferHdl->nonBlockXferState = kNafeNonBlockXferState_dataReceived;

            if (xferHdl->sampleMode == kNafeSampleMode_scsrNonBlock)
            {
                NAFE_HAL_readSpiRxd(devHdl->halHdl, xferHdl->pResult, \
                    (uint32_t)xferHdl->adcResolutionBits, 1u);
            }
            else if (xferHdl->sampleMode == kNafeSampleMode_sccrNonBlock)
            {
                if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
                {
                    pResult = (uint32_t *)xferHdl->pResult + xferHdl->contSampleCnt;
                }
                else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
                {
                    pResult = (uint16_t *)xferHdl->pResult + xferHdl->contSampleCnt;
                }
                NAFE_HAL_readSpiRxd(devHdl->halHdl, \
                    pResult, (uint32_t)xferHdl->adcResolutionBits, 1u);
            }
            else if (xferHdl->sampleMode == kNafeSampleMode_mcmrNonBlock \
                  || xferHdl->sampleMode == kNafeSampleMode_mccrNonBlock)
            {
                uint32_t pResultArrayBaseOffset;
                uint32_t pResultArrayContOffset;
                uint32_t u32Result[NAFE_CHN_AMT_MAX];

                if (xferHdl->sampleMode == kNafeSampleMode_mcmrNonBlock)
                {
                    pResultArrayBaseOffset = 1u;
                    pResultArrayContOffset = 0u;
                }
                else    /* kNafeSampleMode_mccrNonBlock */
                {
                    pResultArrayBaseOffset = xferHdl->contSampleAmt;
                    pResultArrayContOffset = xferHdl->contSampleCnt;
                }

                if (devHdl->readResultFrameMode == kNafeReadResult_singleFrame \
                && devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onSequencer)
                {
                    NAFE_HAL_readSpiRxd(devHdl->halHdl, u32Result, \
                        (uint32_t)xferHdl->adcResolutionBits, xferHdl->chnAmt);

                    for (uint32_t i = 0u; i < xferHdl->chnAmt; i++)
                    {
                        if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
                        {
                            *((uint32_t *)xferHdl->pResult + pResultArrayBaseOffset * i + pResultArrayContOffset) = u32Result[i];
                        }
                        else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
                        {
                            *((uint16_t *)xferHdl->pResult + pResultArrayBaseOffset * i + pResultArrayContOffset) = *((uint16_t *)u32Result + i);
                        }
                    }

                    roundSampleEnd = true;
                }
                else
                {
                    NAFE_HAL_readSpiRxd(devHdl->halHdl, u32Result, \
                        (uint32_t)xferHdl->adcResolutionBits, 1);

                    if (xferHdl->adcResolutionBits == kNafeRegDataSize_24bits)
                    {
                        *((uint32_t *)xferHdl->pResult + pResultArrayBaseOffset * xferHdl->chnSampleCnt + pResultArrayContOffset) = u32Result[0];
                    }
                    else if (xferHdl->adcResolutionBits == kNafeRegDataSize_16bits)
                    {
                        *((uint16_t *)xferHdl->pResult + pResultArrayBaseOffset * xferHdl->chnSampleCnt + pResultArrayContOffset) = *((uint16_t *)u32Result);
                    }

                    if (xferHdl->chnSampleCnt < xferHdl->chnAmt - 1u)
                    {
                        xferHdl->chnSampleCnt++;
                        if (devHdl->sysConfig->readyPinSeqMode == kNafeReadyPinSeqMode_onConversion)
                        {
                            xferHdl->nonBlockXferState = kNafeNonBlockXferState_waitForReady;
                        }
                        else
                        {
                            xferHdl->nonBlockXferState = kNafeNonBlockXferState_ready;
                            readResult = true;
                        }
                    }
                    else
                    {
                        roundSampleEnd = true;
                        xferHdl->chnSampleCnt = 0u;
                    }
                }
            }

            if (xferHdl->sampleMode == kNafeSampleMode_sccrNonBlock
             ||(xferHdl->sampleMode == kNafeSampleMode_mccrNonBlock && roundSampleEnd))
            {
                xferHdl->contSampleCnt++;
                if (xferHdl->contSampleCnt < xferHdl->contSampleAmt)
                {
                    xferHdl->nonBlockXferState = kNafeNonBlockXferState_waitForReady;
                }
                else
                {
                    contSampleEnd = true;
                }
            }

            /* Pull CS high. */
            if (devHdl->readResultFrameMode == kNafeReadResult_multiFrame \
            || (devHdl->readResultFrameMode == kNafeReadResult_singleFrame \
            && (devHdl->sysConfig->triggerMode == kNafeTrigger_spiCmd \
            && (xferHdl->sampleMode == kNafeSampleMode_scsrNonBlock \
            || (xferHdl->sampleMode == kNafeSampleMode_mcmrNonBlock && roundSampleEnd))))
            || contSampleEnd)
            {
                NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
            }

            if (readResult)
            {
                NAFE_readResult(devHdl, xferHdl);
                xferHdl->nonBlockXferState = kNafeNonBlockXferState_rxClkSending;
            }

            if (xferHdl->nonBlockXferState == kNafeNonBlockXferState_dataReceived)
            {
                NAFE_HAL_configSpiInterrupt(devHdl->halHdl, 0u);
                NAFE_HAL_configReadyPinInterrupt(devHdl->halHdl, false);

                /* Terminate continuous sample. */
                if (contSampleEnd)
                {
                    NAFE_terminateSample(devHdl, xferHdl);
                }

                 NAFE_adcCode2Voltage(devHdl, xferHdl);
                xferHdl->nonBlockXferState = kNafeNonBlockXferState_done;
            }
            break;
        default:
            break;
    }
}

/* DMA major loop done IRQ handle. */
void NAFE_dmaIrqHandle(NAFE_devHdl_t *devHdl, NAFE_xferHdl_t *xferHdl)
{
    NAFE_HAL_dmaIrqHandle(devHdl->halHdl, xferHdl);

    if (xferHdl->nonBlockXferState == kNafeNonBlockXferState_done)
    {
        NAFE_HAL_driveSpiCsPin(devHdl->halHdl, kNafeHalSpiCs_high);
        NAFE_terminateSample(devHdl, xferHdl);
        NAFE_adcCode2Voltage(devHdl, xferHdl);
    }
}
