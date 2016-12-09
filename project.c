//*****************************************************************************
//
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.2.111 of the Tiva Firmware Development Package.
//
//*****************************************************************************

/*
    This program performs multiplexer-demultiplexer functions and muxes data from multiple UART ports onto a CAN bus
    for de-multiplexing by another processor.

    (c) 2016, Abhimanyu Ghosh
 */

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/can.h"

// #define GPIO_TEST   1

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

typedef enum {
    UART5,
    UART6,
    UART7
} uart_enum;

void can0_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock GPIO_CR register with this magic value
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

    GPIOPinConfigure(GPIO_PF0_CAN0RX);
    GPIOPinConfigure(GPIO_PF3_CAN0TX); 
    GPIOPinTypeCAN(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    CANInit(CAN0_BASE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);

    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    
    CANEnable(CAN0_BASE);
}

void can0_uart_relay(uart_enum u, uint8_t byte_to_send)
{
    uint8_t data = byte_to_send;

    tCANMsgObject sCANMessage;

    sCANMessage.ui32MsgIDMask = 0;
    sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessage.ui32MsgLen = 1;
    sCANMessage.pui8MsgData = &data;

    switch(u)
    {
        case UART5:
            sCANMessage.ui32MsgID = 0x60000001;
            break;
        case UART6:
            sCANMessage.ui32MsgID = 0x60000003;
            break;
        case UART7:
            sCANMessage.ui32MsgID = 0x60000005;
            break;
    }

    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);
}

void uart5_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));

    GPIOPinConfigure(GPIO_PJ2_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);

    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART5));

    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void uart6_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinConfigure(GPIO_PD4_U6RX);
    GPIOPinConfigure(GPIO_PD5_U6TX);

    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));

    UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    IntEnable(INT_UART6);
    UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
}

void uart7_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);

    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));

    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    IntEnable(INT_UART7);
    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
}

void UART5IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART5_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART5_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART5_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        unsigned char c = UARTCharGetNonBlocking(UART5_BASE);
        // UARTCharPutNonBlocking(UART5_BASE, c);
        can0_uart_relay(UART5, c);
    }
}

void UART6IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART6_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART6_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART6_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        unsigned char c = UARTCharGetNonBlocking(UART6_BASE);
        // UARTCharPutNonBlocking(UART6_BASE, c);
        can0_uart_relay(UART6, c);
    }
}

void UART7IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART7_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART7_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART7_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        unsigned char c = UARTCharGetNonBlocking(UART7_BASE);
        // UARTCharPutNonBlocking(UART7_BASE, c);
        can0_uart_relay(UART7, c);
    }
}

volatile uint8_t g_bRXFlag_uart5;
volatile uint8_t g_bErrFlag_uart5;

volatile uint8_t g_bRXFlag_uart6;
volatile uint8_t g_bErrFlag_uart6;

volatile uint8_t g_bRXFlag_uart7;
volatile uint8_t g_bErrFlag_uart7;

void CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag_uart5 = 1;
        g_bErrFlag_uart6 = 1;
        g_bErrFlag_uart7 = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    //
    else if(ui32Status == 2)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag_uart5 = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag_uart5 = 0;
    }

    else if(ui32Status == 3)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 3);

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag_uart6 = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag_uart6 = 0;
    }

    else if(ui32Status == 4)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 4);

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag_uart7 = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag_uart7 = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
        CANIntClear(CAN0_BASE, ui32Status);
    }
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    g_bRXFlag_uart5 = 0;
    g_bErrFlag_uart5 = 0;

    g_bRXFlag_uart6 = 0;
    g_bErrFlag_uart6 = 0;

    g_bRXFlag_uart7 = 0;
    g_bErrFlag_uart7 = 0;

    uart5_setup();
    uart6_setup();
    uart7_setup();

    can0_init();

    uint8_t can_rx_data_uart5[8];
    uint8_t can_rx_data_uart6[8];
    uint8_t can_rx_data_uart7[8];

    tCANMsgObject sMsgObjectRx_uart5;
    sMsgObjectRx_uart5.ui32MsgID = 0xC0000002;
    sMsgObjectRx_uart5.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart5.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart5.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 2, &sMsgObjectRx_uart5, MSG_OBJ_TYPE_RX);

    tCANMsgObject sMsgObjectRx_uart6;
    sMsgObjectRx_uart6.ui32MsgID = 0xC0000004;
    sMsgObjectRx_uart6.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart6.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart6.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 3, &sMsgObjectRx_uart6, MSG_OBJ_TYPE_RX);

    tCANMsgObject sMsgObjectRx_uart7;
    sMsgObjectRx_uart7.ui32MsgID = 0xC0000006;
    sMsgObjectRx_uart7.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart7.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart7.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 4, &sMsgObjectRx_uart7, MSG_OBJ_TYPE_RX);

    // GPIO Setup for LED Blinky:
    #ifdef GPIO_TEST
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
        GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);
    #endif

    while(1)
    {
        if(g_bRXFlag_uart5 > 0)
        {
            g_bRXFlag_uart5 = 0;
            sMsgObjectRx_uart5.pui8MsgData = can_rx_data_uart5;
            CANMessageGet(CAN0_BASE, 2, &sMsgObjectRx_uart5, 0);
            UARTCharPutNonBlocking(UART5_BASE, can_rx_data_uart5[0]);            
        }

        if(g_bRXFlag_uart6 > 0)
        {
            g_bRXFlag_uart6 = 0;
            sMsgObjectRx_uart6.pui8MsgData = can_rx_data_uart6;
            CANMessageGet(CAN0_BASE, 3, &sMsgObjectRx_uart6, 0);
            UARTCharPutNonBlocking(UART6_BASE, can_rx_data_uart6[0]);
        }

        if(g_bRXFlag_uart7 > 0)
        {
            g_bRXFlag_uart7 = 0;
            sMsgObjectRx_uart7.pui8MsgData = can_rx_data_uart7;
            CANMessageGet(CAN0_BASE, 4, &sMsgObjectRx_uart7, 0);
            UARTCharPutNonBlocking(UART7_BASE, can_rx_data_uart7[0]);
        }

        #ifdef GPIO_TEST
            ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);
            ROM_SysCtlDelay(5000000);
            ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0);
            ROM_SysCtlDelay(5000000);
        #endif
    }
}
