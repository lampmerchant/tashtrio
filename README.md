# TashTrio

Firmware that emulates a trio of devices for Apple's ADB bus: a keyboard, a mouse, and a Global Village TelePort A300 ADB modem.  The emulated keyboard and mouse are controlled by a PS/2 keyboard and mouse, the emulated modem is connected to a two-wire UART.

## Special Keys

The menu key on the PS/2 keyboard is used as a modifier key which can be held and combined with the following keys:

| Key   | Function                       |
| ----- | ------------------------------ |
| F1    | Change UART baud rate to 300.  |
| F2    | Change UART baud rate to 1200. |
| F3    | Change UART baud rate to 2400. |
| F4    | Change UART baud rate to 4800.  Note that this may cause the device connected to the UART to send faster than the Macintosh can receive, which will result in undefined behavior. |
| Pause | Send power key keypress.  Note that this cannot be used to start up the Macintosh, as the microcontroller requires power to be able to read the PS/2 keyboard. |

## Caveats

PS/2 mice require their host to send a command byte to start the flow of mouse movement data.  TashTrio firmware sends this byte on startup only, meaning that the PS/2 mouse must be plugged in at powerup time or it will appear dead.  Because some newer Macintosh computers are known to supply power to the ADB port even when powered down, it is necessary to connect the PS/2 mouse before connecting to the ADB.

## Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB.  Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
