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


## Supported kits (make variable 'TARGET')
- [PSoC 6 Wi-Fi Bluetooth prototyping kit](https://www.cypress.com/CY8CPROTO-062-4343W) (`CY8CPROTO-062-4343W`) - Default value of `TARGET`
- [PSoC 6 WiFi Bluetooth pioneer kit](https://www.cypress.com/CY8CKIT-062-WiFi-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC 62S2 Wi-Fi Bluetooth pioneer kit](https://www.cypress.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC 62S1 Wi-Fi Bluetooth pioneer kit](https://www.cypress.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC 62S1 Wi-Fi Bluetooth pioneer kit](https://www.cypress.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- CYSBSYSKIT-01 Rapid IoT Connect Platform RP01 Feather Kit (`CYSBSYSKIT-01`)
- Rapid IoT Connect Developer Kit (`CYSBSYSKIT-DEV-01`)

## Hardware Setup

This application uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

The application running on a PSoC 6 MCU kit and the test setup are shown below:

![Mfg test architecture](mfg-test-architecture.png)

Note: The PSoC 6 BLE Pioneer Kit (CY8CKIT-062-BLE) and the PSoC 6 WiFi-BT Pioneer Kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox software requires KitProg3. Before using this application, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the Firmware Loader GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".


## Software Setup

1. This application requires WMBT Tool running on a windows PC and uses UART port for communication with target. The pre-built executables for WMBT Tool are available in btsdk-utils/wmbt/bin/ directory, which sync from [btsdk-utils](https://github.com/cypresssemiconductorco/btsdk-utils). and user guide is in [Bluetooth Manufacturing Test Tool](https://www.cypress.com/file/298091/download).

2. IQxel tool as transmitter to send fixed count test packet which to ensure whatever is sent from the transmitter would received by the receiver, without any error.

3. Using Sniffer to ensure whatever is test packet is in same transmit channel, packet length and data patterns from transmitter.

4. Better to test it in the shield room to avoid air interference.

## Using the Application

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or use **File** > **New** > **ModusToolbox Application**). This launches the [Project Creator](http://www.cypress.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit, use the [Library Manager](https://www.cypress.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. Optionally, change the suggested **New Application Name**.

5. Enter the local path in the **Application(s) Root Path** field to indicate where the application needs to be created.

   Applications that can share libraries can be placed in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox User Guide](https://www.cypress.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox install directory}/ide_{version}/docs/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox provides the Project Creator as both a GUI tool and a command line tool to easily create one or more ModusToolbox applications. See the "Project Creator Tools" section of the [ModusToolbox User Guide](https://www.cypress.com/ModusToolboxUserGuide) for more details.

Alternatively, you can manually create the application using the following steps:

1. Download and unzip this repository on to your local machine, or clone the repository.

2. Open a CLI terminal and navigate to the application folder.

   On Linux and macOS, you can use any terminal application. On Windows, open the **modus-shell** app from the Start menu.

   **Note:** The cloned application contains a default BSP file (*TARGET_xxx.mtb*) in the *deps* folder. Use the [Library Manager](https://www.cypress.com/ModusToolboxLibraryManager) (`make modlibs` command) to select and download a different BSP file, if required. If the selected kit does not have the required resources or is not [supported](#supported-kits-make-variable-target), the application may not work.

3. Import the required libraries by executing the `make getlibs` command.

Various CLI tools include a `-h` option that prints help information to the terminal screen about that tool. For more details, see the [ModusToolbox User Guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

<details><summary><b>In third-party IDEs</b></summary>

1. Follow the instructions from the **In command-line interface (CLI)** section to create the application, and import the libraries using the `make getlibs` command.

2. Export the application to a supported IDE using the `make <ide>` command.

   For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox User Guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox install directory}/docs_{version}/mtb_user_guide.pdf*.

3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

</details>

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
