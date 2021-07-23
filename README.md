# PSoC 6 MCU: Bluetooth Manufacturing Test Application for FreeRTOS

The Bluetooth Manufacturing Test Application is used to validate the Bluetooth Firmware and RF performance of Cypress SoC Bluetooth BR/EDR/LE devices.

The Bluetooth MFG Application acts as a transport layer between the host "WMBT tool" and Bluetooth Firmware. Mfg Test Application receive commands from the WMBT tool and forwards them to the Bluetooth firmware. The Bluetooth MFG Application also relays the response received back from Bluetooth firmware.

There are 2 parts of the functions for testing.

1. On the PC side, Application (WMBT) running on PC that will send HCI commands and receive HCI events to PSoC board

   1. The SIG defined BLE testing with below 3 standard HCI commands, to test the LE HW functionalities.

      1. LE Transmitter Test Command

      2. LE Receiver Test Command

      3. LE Test End Command

2. On the PSoC side, there will be another application(MFG app) that will do

   1. Bluetooth FW download. This is to download Bluetooth firmware into the controller, so it can have proper controller HW and RF configured.

   2. Route the HCI commands packets received from PC app to another PSoC UART that with BT controller connected.

   2. App received the HCI event from BT controller and route to PSoC UART to host PC application.

## Requirements
- [ModusToolbox™ IDE](https://www.cypress.com/products/modustoolbox-software-environment) v2.2
- Programming Language: C
- Supported Toolchains: Arm® GCC, IAR
- Associated Parts: All [PSoC 6 MCU](http://www.cypress.com/PSoC6) parts

## Dependent assets
- [Bluetooth FreeRtos Library](https://github.com/cypresssemiconductorco/bluetooth-freertos) - Contains bluetooth firmware


## Validated Kits
- [PSoC 6 Wi-Fi BT Prototyping Kit](https://www.cypress.com/CY8CPROTO-062-4343W) (CY8CPROTO-062-4343W) - Default target

## Hardware Setup

This application uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

The application running on a PSoC 6 MCU kit and the test setup are shown below:

![Mfg test architecture](mfg-test-architecture.png)

Note: The PSoC 6 BLE Pioneer Kit (CY8CKIT-062-BLE) and the PSoC 6 WiFi-BT Pioneer Kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox software requires KitProg3. Before using this application, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the Firmware Loader GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".


## Software Setup

1. This application requires WMBT Tool running on a windows PC and uses UART port for communication with target. The pre-built executables for WMBT Tool are available in wmbt-tool-bin/ directory, which sync from [btsdk-utils](https://github.com/cypresssemiconductorco/btsdk-utils). and user guide is in [Bluetooth Manufacturing Test Tool](https://www.cypress.com/file/298091/download).

2. IQxel tool as transmitter to send fixed count test packet which to ensure whatever is sent from the transmitter would received by the receiver, without any error.

3. Using Sniffer to ensure whatever is test packet is in same transmit channel, packet length and data patterns from transmitter.

4. Better to test it in the shield room to avoid air interference.

## Using the Application


### In Command-line Interface (CLI) on the build machine:


#### Building app

On Linux and macOS, you can use any terminal application. On Windows, open the modus-shell app from the Start menu.

- `git clone https://devops-git.aus.cypress.com/repo-staging/mtb-anycloud-bluetooth-mfg-tester`
- `cd mtb-anycloud-bluetooth-mfg-tester`
- `make getlibs`
- `make program TARGET=CY8CPROTO-062-4343W TOOLCHAIN=GCC_ARM`


## Operation

1. Go to WMBT tool directory

2. Reset the Board by pressing SW1 button

3. Run the command on Windows Host for the proper BT Chip on target board.

4. Observe the output of the command

5. List of wmbt commands with BLE function which can be retrieved by typing --help
   Partial output of the command and display is below.

		Usage: wmbt reset COMx
		Usage: wmbt le_receiver_test COMx <rx_frequency>
		Usage: wmbt le_transmitter_test COMx <tx_frequency> <data_length> <data_pattern>
		Usage: wmbt le_test_end COMx
