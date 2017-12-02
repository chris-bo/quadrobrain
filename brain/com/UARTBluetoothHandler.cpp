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
}

UARTBluetoothHandler::~UARTBluetoothHandler() {

}

void UARTBluetoothHandler::initialize() {

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void UARTBluetoothHandler::startRX() {

    HAL_UART_Receive_IT(uart, RxBuffer, 1);

}

void UARTBluetoothHandler::sendTXBuffer(uint16_t byte_count) {

    HAL_UART_Transmit_IT(uart, TxBuffer, byte_count);

}
