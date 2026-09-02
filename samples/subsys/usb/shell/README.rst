.. zephyr:code-sample:: usb-shell
   :name: USB shell
   :relevant-api: usbd_api

   Use shell commands to interact with USB device stack.

Overview
********

The sample enables new experimental USB device support and the shell function.
It is primarily intended to aid in the development and testing of USB controller
drivers and new USB support.

Building and flashing
*********************

The shell can be built with host and/or device stack support by selecting the
:ref:`application-file-suffixes`, where ``-DFILE_SUFFIX=<mode>`` selects
``prj_<mode>.conf``.

Assuming the board has a supported USB device controller, the example can be
built like:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/shell
   :board: reel_board
   :gen-args: -DFILE_SUFFIX=device
   :goals: flash
   :compact:

Assuming the board has a supported USB host controller, the example can be
built like:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/shell
   :board: nrf52840dk/nrf52840
   :shield: sparkfun_max3421e
   :gen-args: -DFILE_SUFFIX=host
   :goals: flash
   :compact:

Assuming the board has a supported USB device and host controller,
the example can be built like:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/shell
   :board: nrf52840dk/nrf52840
   :shield: sparkfun_max3421e
   :gen-args: -DFILE_SUFFIX=device_and_host
   :goals: flash
   :compact:

It is theoretically possible to build USB support using virtual USB controllers
for all platforms, eventually the devicetree overlay has to be adjusted slightly if
the platform has already defined or not ``zephyr_uhc0`` or ``zephyr_udc0`` nodelabels.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/shell
   :board: nrf52840dk/nrf52840
   :gen-args: -DFILE_SUFFIX=device_and_host -DDTC_OVERLAY_FILE=virtual.overlay
   :goals: flash
   :compact:

Sample shell interaction
========================

.. code-block:: console

   *** Booting Zephyr OS build v4.2.0-1588-g83f1bd7341de ***
   uart:~$ usbd defcfg
   dev: added default string descriptors
   dev: register FS loopback_0
   dev: register HS loopback_0
   dev: USB initialized
   uart:~$ usbh init
   host: USB host initialized
   uart:~$ usbh enable
   host: USB host enabled
   uart:~$ usbh bus resume
   host: USB bus resumed
   uart:~$ usbd enable
   dev: USB enabled
   [160:04:13.870,000] <inf> usb_loopback: Enable loopback_0
   uart:~$ usbh device list
   1
   uart:~$ usbh device descriptor device 1
   host: USB device with address 1
   Device Descriptor:
     bLength                        18
     bDescriptorType                 1
     bcdUSB                       2.00
     bDeviceClass                  239
     bDeviceSubClass                 2
     bDeviceProtocol                 1
     bMaxPacketSize0                64
     idVendor                   0x2FE3
     idProduct                  0xFFFF
     bcdDevice                    4.02
     iManufacturer                   1
     iProduct                        2
     iSerial                         3
     bNumConfigurations              1
   uart:~$


Sample DFU interaction
======================

The USB host shell sample includes commands for Device Firmware Upgrade (DFU) operations when interacting with attached USB devices.

.. note::
   DFU shell support depends on :kconfig:option:`CONFIG_USBH_DFU_CLASS` being enabled.

   For all DFU commands below:

   * **1st argument** (``<alt>``): Alternate function setting number, usually 0.

Entering DFU Mode
-----------------

To switch a device from DFU-Runtime mode to DFU mode, use the following command:

.. code-block:: console

   usbh device dfurt_enter_dfu


Firmware Upload and Download
----------------------------

Once the device is in DFU mode, you can upload (read) or download (write) firmware data:

.. code-block:: console

   usbh device dfu_upload 0

   usbh device dfu_dnload 0 firmware0.dfu


How to prepare filesystem for NOR Flash for EK_RA8M1
----------------------------------------------------

NOTE: Using only a 1M filesystem because flashing is quite slow.
(A) Create the 'littlefs_dir' directory and create/copy some FW files into it..
  Files with a '.dfu' extension will have to pass the DFU suffix check.
  Files without the '.dfu' extension will be uploaded as they are.
  You can add a DFU suffix to any file by running the command:
  `dfu-suffix -a <my_fw_filename.dfu>`
(B) Command to create a 1M filesystem, placed at offset 0x40000 in NOR flash:
  The arguments must match the .dts settings in this .overlay file.
  `mklittlefs -c littlefs_dir -b 0x40000 -p 16 -s 0x100000 littlefs_dfu_1M.bin`
(C) Disconnect the board, populate J16, plug the board back in, and use the
  following command to load the filesystem onto the NOR flash:
  `rfp-cli -d ra -tool jlink -external-loader EK-RA8M1 -bin 0x90040000 littlefs_dfu_1M.bin -e -p -v`
(D) When flashing is done, unplug the board, remove J16, then plug the board back in.
(E) Flash the usb/shell demo as usual
