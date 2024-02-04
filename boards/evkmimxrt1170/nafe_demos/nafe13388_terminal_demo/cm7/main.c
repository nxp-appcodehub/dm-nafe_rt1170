/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void terminalDemoInit(void);
extern void terminalDemo(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n>> i.MXRT1170 & NAFE terminal demo starting...\r\n");

    terminalDemoInit();

    /* Start to sample and transfer. */
    while(1)
    {
        terminalDemo();
    }
}
