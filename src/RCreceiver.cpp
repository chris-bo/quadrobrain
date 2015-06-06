/*
 * RCreceiver.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: bohni
 */

#include "RCreceiver.h"

RCreceiver::RCreceiver(Status* statusPtr, uint8_t defaultPrio,
            TIM_HandleTypeDef* htim)
: Task(statusPtr, defaultPrio) {
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
    RCreceiver_htim = htim;

}

RCreceiver::~RCreceiver() {
}

void RCreceiver::update() {
    if (GET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_NO_SIGNAL)) {
        /* TODO RCreceiver manage rc signal loss
         *
         * 		handle throttle and engines
         * */

        /* disable control*/
        status->rcSignalNick = 0;
        status->rcSignalRoll = 0;
        status->rcSignalYaw = 0;

        /* reduceThrottle to 20% */
        if (status->rcSignalThrottle > 0.2f) {
            status->rcSignalThrottle = 0.2f;
        }
        signalLostTime++;
        if (signalLostTime == 200) {
            /* Disable control and engines */
            status->rcSignalNick = 0;
            status->rcSignalRoll = 0;
            status->rcSignalYaw = 0;
            status->rcSignalEnable = 0;

        }
    } else {
        computeValues();
        signalLostTime = 0;

    }
    resetPriority();
}

void RCreceiver::computeValues() {
    /* raw timer output -> channel control values 0-100% */
    int16_t tmp;
    for (uint8_t i = 0; i < RECEIVER_CHANNELS; i++) {

        tmp = (int16_t) (rawReceiverValues[i] - RC_RECEIVER_MinHighTime_us);
        tmp = tmp
                    / ((RC_RECEIVER_MaxHighTime_us - RC_RECEIVER_MinHighTime_us)
                                / 100);
        if (tmp < 0) {
            tmp = 0;
        } else if (tmp > 100) {
            tmp = 100;
        }
        rawRCvalues[i] = (uint8_t) tmp;
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

    status->rcSignalRoll = (((float) rawRCvalues[0] - 50) / 100);
    status->rcSignalNick = (((float) rawRCvalues[2] - 50) / 100);
    status->rcSignalYaw = (((float) rawRCvalues[3] - 50) / 100);
    status->rcSignalThrottle = ((float) rawRCvalues[1] / 100);
    status->rcSignalLinPoti = ((float) rawRCvalues[5] / 100);
    if (rawRCvalues[4] > 80) {
        status->rcSignalEnable = 1;
    } else {
        status->rcSignalEnable = 0;
    }
    if (rawRCvalues[6] > 80) {
        status->rcSignalSwitch = 1;
    } else {
        status->rcSignalSwitch = 0;
    }

}

void RCreceiver::overrunIRQ() {
    /* Overrun interrupt
     * overrun -> sync -> reset counter
     */
    if (currentChannel == 8) {

    } else {
        SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_NO_SIGNAL);
        RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC);
        currentChannel = 8;
    }
}

void RCreceiver::captureIRQ() {
    /* Input Capture Sequence
     * input capture -> save value -> reset counter
     * */
    RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_NO_SIGNAL);
    if (currentChannel == 8) {
        /* after sync sequence */
        __HAL_TIM_SetCounter(RCreceiver_htim, 0x00);
        HAL_TIM_ReadCapturedValue(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
        currentChannel = 0;
        RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
    } else {
        rawReceiverValues[currentChannel] = (uint16_t) HAL_TIM_ReadCapturedValue(
                    RCreceiver_htim,
                    RC_RECEIVER_INPUT_CHANNEL);
        __HAL_TIM_SetCounter(RCreceiver_htim, 0);
        if (currentChannel == 7) {
            /* all pulses detected and saved */
            /* waiting for overrun interrupt to resync */
            SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
            SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC);
        }
        currentChannel++;

    }

}

void RCreceiver::initialize() {

    /* initialize timer and interrupts
     * needs to be called after MX_TIM4_Init();
     * */
    HAL_TIM_Base_MspInit(RCreceiver_htim);
    __HAL_TIM_ENABLE_IT(RCreceiver_htim, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void RCreceiver::stop() {
    /* stops RC receiver */
    __HAL_TIM_DISABLE_IT(RCreceiver_htim, TIM_IT_UPDATE);
    HAL_TIM_IC_Stop_IT(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
    status->rcSignalNick = 0;
    status->rcSignalRoll = 0;
    status->rcSignalYaw = 0;
    status->rcSignalThrottle = 0;
    status->rcSignalEnable = 0;
    status->rcSignalSwitch = 0;

}
