/*
 * UARTBluetoothHandler.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: chris
 */

#include "UARTBluetoothHandler.h"

UARTBluetoothHandler::UARTBluetoothHandler(Status* statusPtr,
        uint8_t defaultPrio, DiscoveryLEDs* _ledctrl, Led_TypeDef _led,
        UART_HandleTypeDef* huart) :
        RxTxHandler(statusPtr, defaultPrio, _ledctrl, _led) {

    uart = huart;
    UartRXbuf[UART_RXTX_BUFF_SIZE] = {0};
    UartTXbuf[UART_RXTX_BUFF_SIZE] = {0};
    uart_bytes_received = 0;

    initializeBuffers(UartRXbuf, UartTXbuf, &uart_bytes_received,
    UART_RXTX_BUFF_SIZE);

    /* dummy padding variable */
    pad = 0;
}

UARTBluetoothHandler::~UARTBluetoothHandler() {

}

void UARTBluetoothHandler::initialize() {

    taskActive = true;
}

void UARTBluetoothHandler::startRX() {

    if (HAL_UART_GetState(uart) != HAL_UART_STATE_BUSY_RX) {
        /* start receiving */
        *numberReceivedData = 0;
        receptionComplete = false;
        HAL_UART_Receive_IT(uart, RxBuffer, 1);
    }

}

void UARTBluetoothHandler::sendTXBuffer(uint16_t byte_count) {

    HAL_UART_Transmit_IT(uart, TxBuffer, byte_count);

}

//void UARTBluetoothHandler::byteReceived() {
//
//    *numberReceivedData = (uint16_t) (*numberReceivedData + ((uint16_t) 1));
//    /* wait for next byte */
//    HAL_UART_Receive_IT(uart, RxBuffer+*numberReceivedData, 1);
//    check for end of frame and set reception complete
//}

void UARTBluetoothHandler::rxCallback() {

    if (*numberReceivedData == 0) {
        *numberReceivedData = (uint16_t) (RxBuffer[0] + ((uint16_t) 1));
        HAL_UART_Receive_IT(uart, &RxBuffer[1], RxBuffer[0]);

    } else {
        receptionComplete = true;
    }

}
