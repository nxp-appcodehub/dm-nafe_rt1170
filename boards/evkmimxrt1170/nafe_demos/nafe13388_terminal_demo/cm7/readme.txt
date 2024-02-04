Overview
========
The nafe_13388 example demonstrates how to use i.MXRT1170 and nafe13388 to sample
analog signals.

Toolchain supported
===================
- IAR embedded Workbench  8.50.9 or later

Other tools required
===================
- FreeMaster
- A serial terminal, like Tera Term

Hardware requirements
=====================
- Personal Computer
- MIMXRT1170-EVK board
- NAFE13388-EVB
- A micro USB cable
- Power source of +15.4V, -15.4V, +3.75V
- Signal source, like signal generator
- Several dupont wires

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect the MIMXRT1170-EVK and NAFE13388-EVB with some dupont wires:
     --------------------------------------------
    |    MIMXRT1170-EVK      |   NAFE13388-EVB   |
    |  GPIO3 Pin13 - J9-16   |  SYNC   -  J79-4  |
    |  GPIO3 Pin6  - J10-2   |  DRDYB  -  J78-8  |
    |  LPSPI1 CS   - J10-6   |  CSB    -  J79-6  |
    |  LPSPI1 SDO  - J10-8   |  MOSI   -  J78-4  |
    |  LPSPI1 SDI  - J10-10  |  MISO   -  J78-6  |
    |  LPSPI1 CLK  - J10-12  |  SCK    -  J78-2  |
    |  GND         - J10-14  |  GND    -  J1-2&3 |
     --------------------------------------------
2.  Apply power supply on the banana jack connectors of NAFE13388-EVB:
    - AVDD/DVDD = 3.75V on J9, 
    - HVDD = +15.4V on J10 
    - HVSS = -15.4V on J11 
    and turn on the power supply. ~0.4V additional supply on J9, J10 and J11 are
    to account for the voltage drop across Schottky diode.
3.  Provide two differential signals to the NAFE13388-EVB analog inputs with the signal
    generator, such as sine waveforms:
    - Signal 1 positive to J83 AI1P
    - Signal 1 negative to J83 AI1N
    - Signal 2 positive to J83 AI2P
    - Signal 2 negative to J83 AI2N
4.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board. 
5.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
6.  Download the program to the target board.
7.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
The log below shows the output of demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

NAFE Demo for Rev B silicon.
Compiled at xx:xx:xx  Xxx xx xxxx
Supported modes:
1 - SCSR Block (Polling).
2 - SCCR Block (Polling).
3 - MCMR Block (Polling).
4 - MCCR Block (Polling).
5 - SCSR Non-Block (Interrupt).
6 - SCCR Non-Block (Interrupt).
7 - MCMR Non-Block (Interrupt).
8 - MCCR Non-Block (Interrupt).
9 - SCCR DMA.
a - MCCR DMA.
Enter the corresponding character to select a sample mode:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Type in a character (1~a) to select one of the sample modes.
 - If '1', or '3', or '5', or '7' is typed in, the corresponding non-continuous 
   sample routine will be called in an infinite loop. Close the serial terminal and
   open the FreeMaster project "FreeMaster.pmpx" to see the real-time waveform.
 - If '2', or '4', or '6', or '8', or '9', or 'a' is typed in, it will sample
   100 continuous results by default, and the results will printed in the terminal.
