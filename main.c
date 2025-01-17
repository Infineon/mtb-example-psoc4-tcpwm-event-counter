/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 TCPWM Event Counter
 * Example for ModusToolbox.
 *
 * Related Document: See README.md 
 *
 *******************************************************************************
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "stdio.h"

/*******************************************************************************
 *        Constants
 *******************************************************************************/
#define LED_ON               (0u)      /* Value to switch LED ON  */
#define LED_OFF              (!LED_ON) /* Value to switch LED OFF */

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void handle_error(void);
void timer_isr_handler(void);

/*******************************************************************************
 *        Global variables
 *******************************************************************************/
volatile bool its_ten_seconds;

cy_stc_scb_uart_context_t uart_context;

/******************************************************************************
 * timer interrupt configuration structure
 *******************************************************************************/
cy_stc_sysint_t timer_intr_cfg =
{
    .intrSrc = TIMER_IRQ,  /* Source of interrupt signal*/
    .intrPriority = 3UL    /* Interrupt priority */
};

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * System entrance point. This function performs
 *   1. Initializes the BSP.
 *   2. Configures the interrupt.
 *   3. Configures the timer and counter peripherals.
 *   4. Configures the SCB block as UART interface.
 *   5. Prints out "count value" via UART interface.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t event_count;
    char print[4];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* UART initialization status */
    cy_en_scb_uart_status_t uart_init_status;

    /* Initialize TIMER interrupt */
    Cy_SysInt_Init(&timer_intr_cfg, timer_isr_handler);
    NVIC_EnableIRQ(timer_intr_cfg.intrSrc);

    /* Initialize the EVENT_COUNTER */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(EVENT_COUNTER_HW, EVENT_COUNTER_NUM, &EVENT_COUNTER_config))
    {
        /* Handle possible errors */
        CY_ASSERT(0);
    }
    Cy_TCPWM_Enable_Multiple(EVENT_COUNTER_HW, EVENT_COUNTER_MASK);
    Cy_TCPWM_TriggerStart(EVENT_COUNTER_HW, EVENT_COUNTER_MASK);

    /* Initialize the TIMER */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TIMER_HW, TIMER_NUM, &TIMER_config))
    {
        /* Handle possible errors */
        CY_ASSERT(0);
    }
    Cy_TCPWM_Enable_Multiple(TIMER_HW, TIMER_MASK);
    Cy_TCPWM_TriggerStart(TIMER_HW, TIMER_MASK);

    /* Initialize the UART operation */
    uart_init_status = Cy_SCB_UART_Init(UART_HW, &UART_config, &uart_context);
    if (uart_init_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_HW);
    Cy_SCB_UART_PutString(UART_HW, "\r\n\r\nUART initialization complete\r\n");

    for (;;)
    {
        /* TIMER generates interrupt every ten seconds and sets "its_ten_seconds" to true */
        /* For every ten seconds, EVENT_COUNTER count value is read and displayed */
        /* After reading the EVENT_COUNTER count value, count is reset to 0 */
        if (its_ten_seconds == true)
        {
            its_ten_seconds = false;
            event_count = Cy_TCPWM_Counter_GetCounter(EVENT_COUNTER_HW, EVENT_COUNTER_NUM);
            Cy_TCPWM_Counter_SetCounter(EVENT_COUNTER_HW, EVENT_COUNTER_NUM, 0);
            Cy_SCB_UART_PutString(UART_HW, "******************************************************************\n\r");
            Cy_SCB_UART_PutString(UART_HW, "The number of times SW is pressed during previous ten seconds = ");
            sprintf(print,"%ld\n\r", (long int) event_count);
            Cy_SCB_UART_PutString(UART_HW, print);
            Cy_SCB_UART_PutString(UART_HW, "******************************************************************\n\r");
        }
    }
}

/*******************************************************************************
 * Function Name: handle_error
 ********************************************************************************
 * Summary:
 * This function processes unrecoverable errors such as UART peripheral
 * initialization error. In case of such error the system will turn on
 * ERROR_RED_LED and stay in an infinite loop of this function.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts */
    __disable_irq();

    /* Turn on error LED */
    Cy_GPIO_Write(ERROR_RED_LED_PORT, ERROR_RED_LED_NUM, LED_ON);
    while(1u) {}
}

/*******************************************************************************
 * Function Name: timer_isr_handler
 ********************************************************************************
 * Summary:
 *   TIMER interrupt service routine.
 *   This function sets "its_ten_seconds" to true and clears TIMER interrupt.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void timer_isr_handler(void)
{
    its_ten_seconds = true;
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, Cy_TCPWM_GetInterruptStatus(TIMER_HW, TIMER_NUM));
}

/* [] END OF FILE */

