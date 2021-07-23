/******************************************************************************
* File Name:   bt_mfg_test.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bt_mfg_test.h"
#include "bt_transport.h"
#include "bt_bus.h"
#include "bt_firmware.h"
/******************************************************************************
* Macros
******************************************************************************/
#define BAUDRATE_3MBPS       (3000000)
#define BAUDRATE_115KBPS     (115200)

/* Task parameters for MfgTest App Task. */
#define MFG_TEST_TASK_PRIORITY       CY_RTOS_PRIORITY_NORMAL
#define MFG_TEST_TASK_STACK_SIZE     (1024)

#define TRANSPORT_TASK_PRIORITY      CY_RTOS_PRIORITY_NORMAL
#define TRANSPORT_TASK_STACK_SIZE    (4096)

/******************************************************
 *                    Structures
 ******************************************************/
 
/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void bt_mfgtest_task(void *pvParameters);

/******************************************************************************
* Global Variables
******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

bool bt_power_init(cyhal_gpio_t power_pin)
{
    if( CY_RSLT_SUCCESS != cyhal_gpio_init( power_pin,
                                            CYHAL_GPIO_DIR_OUTPUT,
                                            CYHAL_GPIO_DRIVE_PULLUP,
                                            1
                                            ))
    {
        return false;
    }

    cyhal_gpio_write( power_pin, true );
    cy_rtos_delay_milliseconds( 500 );

    return true;
}

/*
 * This is called first after the initialization of FreeRTOS.
 */
void vApplicationDaemonTaskStartupHook()
{
    /* Create the mfg_test Client task. */
    xTaskCreate( bt_mfgtest_task, "Mfg-test task", MFG_TEST_TASK_STACK_SIZE,
                 NULL, MFG_TEST_TASK_PRIORITY, NULL );
}

int main(void)
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if( result != CY_RSLT_SUCCESS )
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
#ifdef MBT_HIGHSPEED
    cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, BAUDRATE_3MBPS );
#else
    cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, BAUDRATE_115KBPS );
#endif

    APP_TRACE_DEBUG("=============================================\n");
    APP_TRACE_DEBUG("BT MfgTest Application\r\n") ;
    APP_TRACE_DEBUG("=============================================\n");

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}

/******************************************************************************
 * Function Name: bt_mfgtest_task
 ******************************************************************************
 * Summary:
 *  Task for handling initialization BT/UART and patchram download
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void bt_mfgtest_task( void *pvParameters )
{
extern void bt_tx_transport_task(cy_thread_arg_t arg);
extern void bt_rx_transport_task(cy_thread_arg_t arg);

extern const uint8_t brcm_patchram_buf[];
extern const int brcm_patch_ram_length;

    APP_TRACE_DEBUG("bt_mfgtest_task\n");

    /* Set I/O to No Buffering */
    setvbuf( stdin, NULL, _IONBF, 0 );

    /* Set I/O to No Buffering */
    setvbuf( stdout, NULL, _IONBF, 0 );

    /* Turn on BT */
    if( !bt_power_init( CYBSP_BT_POWER ) )
    {
        APP_TRACE_DEBUG( "Error Init power pin failed\n" );
        return;
    }

    /* Enable HCI UART */
    if( !bt_bus_init() )
    {
        APP_TRACE_DEBUG( "Error initialising BT bus\n" );
        return;
    }

    /* patchram */
    if( true != bt_firmware_download( brcm_patchram_buf, brcm_patch_ram_length ) )
    {
        APP_TRACE_DEBUG( "Error downloading HCI firmware\n" );
        return;
    }

    APP_TRACE_DEBUG( "start do wmbt/\mbt test\n" );

    bt_trasport_mempool_init();

    /* transport Tx thread - Grab message from PC and pass it over to the controller */
    xTaskCreate( bt_tx_transport_task, "Transport Tx task", TRANSPORT_TASK_STACK_SIZE,
                 NULL, TRANSPORT_TASK_PRIORITY, NULL );

    /* transport Rx thread - Grab message from controller and pass it over to the PC */
    xTaskCreate( bt_rx_transport_task, "Transport Rx task", TRANSPORT_TASK_STACK_SIZE,
                 NULL, TRANSPORT_TASK_PRIORITY, NULL );

    /* Cleanup section for various operations. */
    vTaskDelete( NULL );
}

