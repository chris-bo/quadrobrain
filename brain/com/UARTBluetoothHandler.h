/*
 * UARTBluetoothHandler.h
 *
 *  Created on: Nov 28, 2017
 *      Author: chris
 */

#ifndef UARTBLUETOOTHHANDLER_H_
#define UARTBLUETOOTHHANDLER_H_

#include <com/RxTxHandler.h>

class UARTBluetoothHandler: public RxTxHandler {
public:
    UARTBluetoothHandler(Status* statusPtr, uint8_t defaultPrio,
            DiscoveryLEDs* _ledctrl, Led_TypeDef _led,
            UART_HandleTypeDef* huart);
    virtual ~UARTBluetoothHandler();

    void initialize();
    void startRX();

    void sendTXBuffer(uint16_t byte_count);

    void rxCallback();

private:
    UART_HandleTypeDef* uart;

    uint16_t uart_bytes_received;

    /* dummy padding variable */
    int pad :16;

    uint8_t UartRXbuf[UART_RXTX_BUFF_SIZE];
    uint8_t UartTXbuf[UART_RXTX_BUFF_SIZE];

};

#endif /* UARTBLUETOOTHHANDLER_H_ */
