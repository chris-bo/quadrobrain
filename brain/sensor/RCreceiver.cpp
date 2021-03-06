/*
 * RCreceiver.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: bohni
 */

#include "RCreceiver.h"

RCreceiver::RCreceiver(Status* statusPtr, uint8_t defaultPrio,
        TIM_HandleTypeDef* htim) :
        Task(statusPtr, defaultPrio) {
    currentChannel = 0;
    rawReceiverValues[0] = 0;
    rawReceiverValues[1] = 0;
    rawReceiverValues[2] = 0;
    rawReceiverValues[3] = 0;
    rawReceiverValues[4] = 0;
    rawReceiverValues[5] = 0;
    rawReceiverValues[6] = 0;
    rawReceiverValues[7] = 0;

    rawRCvalues[0] = 0;

    signalLostTime = 0;
    signalLostBuzzerCounter = 0;
    RCreceiver_htim = htim;

    sync = false;
    sequenceComplete = false;
    noSignal = true;
    hadSignal = false;
    error = false;

    /* dummy padding variable */
    pad = 0;

}

RCreceiver::~RCreceiver() {
}

void RCreceiver::update() {

    if (noSignal) {

        status->globalFlags.RCReceiverOk = false;
        status->globalFlags.error = true;
        /* disable control*/
        status->rcSignalNick = 0;
        status->rcSignalRoll = 0;
        status->rcSignalYaw = 0;

        /* reduceThrottle to 20% */
        if (status->rcSignalThrottle > 0.2f) {
            status->rcSignalThrottle = 0.2f;
        }
        signalLostTime++;
        if (hadSignal) {
            if (status->qcSettings.enableBuzzerWarningRCLost) {
                if (signalLostBuzzerCounter++
                        == SCHEDULER_INTERVALL_ms * 1000) {
                    signalLostBuzzerCounter = 0;
                    status->addToneToQueue(&status->buzzerQueue1, BUZZER_B5,
                            300);
                    status->addToneToQueue(&status->buzzerQueue1, BUZZER_PAUSE,
                            100);
                    status->addToneToQueue(&status->buzzerQueue1, BUZZER_B5,
                            300);
                } else {
                    signalLostBuzzerCounter++;
                }
            }
        }  // else ignore signal lost (never had signal)

    } else {
        status->globalFlags.RCReceiverOk = true;
        computeValues();
        signalLostTime = 0;
        signalLostBuzzerCounter = 0;
    }
}

void RCreceiver::computeValues() {
    /* raw timer output -> channel control values 0-100% */
    int16_t tmp;
    for (uint8_t i = 0; i < RECEIVER_CHANNELS; i++) {

        tmp = (int16_t) (rawReceiverValues[i] - RC_RECEIVER_MinHighTime_us);
        tmp = (int16_t) (tmp * 10000
                / (RC_RECEIVER_MaxHighTime_us - RC_RECEIVER_MinHighTime_us));
        if (tmp < 0) {
            tmp = 0;
        } else if (tmp > 10000) {
            tmp = 10000;
        }
        /* add lowpass filter and save rawRCvalue*/
        rawRCvalues[i] = (uint16_t) (rawRCvalues[i] + tmp) / 2;
    }
    /* Channel Configuration:
     *
     * 1: right hor (roll)
     * 2: left vert	(throttle)
     * 3: right vert (nick)
     * 4: left hor (yaw)
     * 5: right switch
     * 6: linear control
     * 7: left switch
     * 8: ---
     *
     */

    status->rcSignalRoll = (((float) (rawRCvalues[0]) / 10000.0f) - 0.5f);
    status->rcSignalNick = (((float) rawRCvalues[2]) / 10000.0f) - 0.5f;
    status->rcSignalYaw = (((float) rawRCvalues[3]) / 10000.0f) - 0.5f;
    status->rcSignalThrottle = (float) (rawRCvalues[1]) / 10000.0f;
    status->rcSignalLinPoti = (float) (rawRCvalues[5]) / 10000.0f;
    if (rawRCvalues[4] > 8000) {
        status->rcSignalEnable = 1;
    } else {
        status->rcSignalEnable = 0;
    }
    if (rawRCvalues[6] > 8000) {
        status->rcSignalSwitch = 1;
    } else {
        status->rcSignalSwitch = 0;
    }

    /* set had signal */
    hadSignal = true;

}

void RCreceiver::overrunIRQ() {
    /* Overrun interrupt
     * overrun -> sync -> reset counter
     */
    if (currentChannel == 8) {

    } else {
        noSignal = true;
        currentChannel = 8;
    }
}

void RCreceiver::captureIRQ() {
    /* Input Capture Sequence
     * input capture -> save value -> reset counter
     * */
    if (currentChannel == 8) {
        /* after sync sequence */
        __HAL_TIM_SET_COUNTER(RCreceiver_htim, 0x00);
        HAL_TIM_ReadCapturedValue(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
        currentChannel = 0;
    } else {
        rawReceiverValues[currentChannel] =
                (uint16_t) HAL_TIM_ReadCapturedValue(RCreceiver_htim,
                RC_RECEIVER_INPUT_CHANNEL);
        __HAL_TIM_SET_COUNTER(RCreceiver_htim, 0);
        currentChannel++;
        if (currentChannel == 8) {
            /* all pulses detected and saved */
            /* waiting for overrun interrupt to resync */
            /* reset no signal flag*/
            noSignal = false;
        }
    }
}

void RCreceiver::initialize() {

    /* initialize timer and interrupts
     * needs to be called after MX_TIM4_Init();
     * */

    /* enable update interrupt to get overrun and sync info*/
    __HAL_TIM_ENABLE_IT(RCreceiver_htim, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);

    taskActive = true;

}

void RCreceiver::kill() {
    /* stops RC receiver */
    __HAL_TIM_DISABLE_IT(RCreceiver_htim, TIM_IT_UPDATE);
    HAL_TIM_IC_Stop_IT(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
    status->rcSignalNick = 0;
    status->rcSignalRoll = 0;
    status->rcSignalYaw = 0;
    status->rcSignalThrottle = 0;
    status->rcSignalEnable = 0;
    status->rcSignalSwitch = 0;

    currentChannel = 0;
    rawReceiverValues[0] = 0;
    rawReceiverValues[1] = 0;
    rawReceiverValues[2] = 0;
    rawReceiverValues[3] = 0;
    rawReceiverValues[4] = 0;
    rawReceiverValues[5] = 0;
    rawReceiverValues[6] = 0;
    rawReceiverValues[7] = 0;

    for (uint8_t i = 0; i < RECEIVER_CHANNELS; i++) {
        rawRCvalues[i] = 0;
    }

    signalLostTime = 0;

    reset();
    taskActive = false;
    status->globalFlags.RCReceiverOk = false;
}
