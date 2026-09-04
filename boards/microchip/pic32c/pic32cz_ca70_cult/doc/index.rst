.. zephyr:board:: pic32cz_ca70_cult

Overview
********

The PIC32CZ CA70 Curiosity Ultra development board is a hardware platform
to evaluate the Microchip PIC32CZ CA70 microcontroller, and the
development board part number is EV56T44A/EA60E74A. This kit provides a comprehensive
set of features that allow users to explore the PIC32CZ CA70 peripherals
and gain insight into integrating the device into their own designs. Users
can expand functionality through Arduino Uno R3, mikroBUS, or Xplained Pro
compatible expansion boards.

Hardware
********

- 144-pin TQFP PIC32CZ2051CA70144 microcontroller
- Arm® Cortex®-M7 Microcontroller running at up to 300 MHz
- 32.768 kHz crystal oscillator
- 12 MHz crystal oscillator
- 2 MiB flash memory and 512 KiB of RAM
- Two user LEDs
- Two mechanical user push buttons
- One reset button
- Micro USB interface (Type-AB)
- Virtual COM port (VCOM)
- Data Gateway Interface (DGI)
- Programming and debugging of on-board PIC32CZ CA70 through Serial Wire Debug (SWD) and on-board PKoB4 debugger
- Arduino UNO R3 Shield Headers
- Two Xplained Pro Standard Extension Headers
- mikroBUS™ Socket
- Up to two Controller Area Network (CAN) interfaces
- Ethernet MAC (GMAC) interface
- Graphics interface
- On-board temperature sensor
- QSPI flash memory
- Serial EEPROM

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

The `PIC32CZ CA70 Curiosity Ultra User Guide`_ has detailed information about board connections.

Programming & Debugging
***********************

.. zephyr:board-supported-runners::

Flash Using OpenOCD
===================

The PIC32CZ CA70 Curiosity Ultra board has an on-board PKoB4 debugger that presents itself as a
CMSIS-DAP device, so no external debug probe is required. To flash the board, follow the steps below:

1. Connect the Board

   - Connect the **DEBUG USB** port on the board to your host machine via USB. This both
     powers the board and exposes the on-board PKoB4 debugger.

2. Build the Application

   You can build a sample Zephyr application, such as **Blinky**, using the ``west`` tool.
   Run the following commands from your Zephyr workspace:

   .. code-block:: console

      west build -b pic32cz_ca70_cult -p -s samples/basic/blinky

   This will build the Blinky application for the ``pic32cz_ca70_cult`` board.

3. Flash the Device

   Once the build completes, flash the firmware using:

   .. code-block:: console

      west flash

   This uses the ``openocd`` runner configured for this board to flash the application
   through the on-board PKoB4 (CMSIS-DAP) debugger.

4. Observe the Result

   After flashing, **LED0** on the board should start **blinking**, indicating that the
   application is running successfully.


Debug Using OpenOCD: GDB Command-Line
=====================================

To debug the board using OpenOCD, follow the steps below:

1. Build the Application

   You can build a sample Zephyr application, such as **Blinky**, using the ``west`` tool.
   Run the following commands from your Zephyr workspace:

   .. code-block:: console

      west build -b pic32cz_ca70_cult -p -s samples/basic/blinky

   This will build the Blinky application for the ``pic32cz_ca70_cult`` board.

2. Debug the Application

   To debug the application, run the following command from your Zephyr workspace:

   .. code-block:: console

      west debug

   .. note::
      ``Debugging with OpenOCD/GDB: Halting at `main` and Setting Breakpoints``

      .. code-block:: sh

         (gdb) break main
         (gdb) break main.c:42         # Optional: set breakpoint at line 42 in main.c
         (gdb) continue                # Start execution, halts at first breakpoint
         (gdb) info breakpoints        # List all breakpoints
         (gdb) delete 1                # Delete breakpoint number 1
         (gdb) delete                  # Delete all breakpoints
         (gdb) clear main.c:42         # Clear breakpoint at specific location
         (gdb) continue                # Resume execution

Debug Using OpenOCD: VSCode
===========================

To debug using VSCode, create or edit the **.vscode/launch.json** file in your project directory with the following configuration:

.. code-block:: json

   {
      "version": "0.2.0",
      "configurations": [
         {
               "name": "Debug (OpenOCD, PIC32CZ2051CA70144)",
               "type": "cortex-debug",
               "request": "launch",
               "servertype": "openocd",
               "device": "PIC32CZ2051CA70144",
               "interface": "swd",
               "runToEntryPoint": "main",
               "showDevDebugOutput": "none",
               "cwd": "${workspaceFolder}",
               "executable": "${workspaceFolder}/zephyr/build/zephyr/zephyr.elf",

               //OpenOCD settings
               "serverpath": "***/openocd/bin/openocd",
               "configFiles": [
                  "***/zephyr/boards/microchip/pic32c/pic32cz_ca70_cult/support/openocd_pic32cz_ca70_cult.cfg"
               ],

               //Zephyr SDK toolchain paths
               "gdbPath": "***/zephyr-sdk-x.xx.x/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb.exe",
               "objdumpPath": "***/zephyr-sdk-x.xx.x/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe",
         }
      ]
   }

.. note::

   - **serverpath**: Specify the full path to the OpenOCD executable if it is not in your system's PATH.

   - **gdbPath**: Specify the full path to the GDB executable if it is not in your system's PATH.

   - **objdumpPath**: Specify the full path to the objdump executable if it is not in your system's PATH.

Debugging Steps in VSCode
-------------------------

1. Start debugging as shown above.
2. Open the samples/basic/blinky/src/main.c in VSCode.
3. Set breakpoints in your source files by clicking next to the line numbers.
4. Press **F5** or click the green "Start Debugging" button in VSCode.
5. The debugger will connect, load your program, and halt at **main**.
6. Use the VSCode debug controls to continue, step, or inspect variables.

References
**********

PIC32CZ CA70 Product Page:
   https://www.microchip.com/en-us/product/PIC32CZ2051CA70144

PIC32CZ CA70 Curiosity Ultra Development Board Page:
   https://www.microchip.com/en-us/development-tool/EV56T44A

.. _PIC32CZ CA70 Curiosity Ultra User Guide:
   https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/UserGuides/PIC32CZ-CA70-Curiosity-Ultra-User-Guide-DS70005572.pdf
