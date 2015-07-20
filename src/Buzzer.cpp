/*
 * Buzzer.cpp
 *
 *  Created on: 26.04.2015
 *      Author: Jonas
 */

#include "Buzzer.h"

Buzzer::Buzzer(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim)
            : Task(statusPtr, defaultPrio) {
    buzzerHtim = htim;
    elapsedLengthBuzzer = 0;
    toneLengthBuzzer = 0;
}

void Buzzer::update() {
    if (toneLengthBuzzer > 0) {
        if (elapsedLengthBuzzer == toneLengthBuzzer) {
            // stop pwm timer
            HAL_TIM_PWM_Stop(buzzerHtim, TIM_CHANNEL_1);
            // reset lengths
            toneLengthBuzzer = 0;
            if (queue.index != queue.currentTone) {
                /* queue not empty */
                playNextTone();
            }
        } else {
            // go one timestep forward
            elapsedLengthBuzzer++;
        }
    }
    resetPriority();
}

void Buzzer::playTone(float frequency, uint16_t length) {
    addToneToQueue(frequency, length);
}

uint16_t Buzzer::calculateReloadValue(float frequency) {
    // nreload = 72Mhz / nprescaler / frequency
    uint32_t tmp = ((HAL_RCC_GetSysClockFreq() / (BUZZER_PRESCALER + 1)) / frequency);
    return (uint16_t) (tmp);
}

Buzzer::~Buzzer() {

}

void Buzzer::addToneToQueue(float frequency, uint16_t length) {

    /* check if timer is running */
    uint8_t timerStopped = 0;
    if ((queue.index == queue.currentTone) && (toneLengthBuzzer == 0)) {
        timerStopped = 1;
    }

    /* goto next address */
    queue.index++;
    if (queue.index == BUZZER_QUEUE_SIZE) {
        queue.index = 0;
    }

    /* add tone */
    queue.frequency[queue.index] = frequency;
    queue.lenght[queue.index] = length;

    /* call playNextTone() if timer is stopped
     */
    if (timerStopped) {
        playNextTone();
    }
}

void Buzzer::playNextTone() {

    /* goto next tone */
    queue.currentTone++;
    if (queue.currentTone == BUZZER_QUEUE_SIZE) {
        queue.currentTone = 0;
    }

    // set length
    toneLengthBuzzer = queue.lenght[queue.currentTone];
    // set pwm timer
    if (queue.frequency[queue.currentTone] > 0) {
        uint16_t temp = calculateReloadValue(queue.frequency[queue.currentTone]);
        __HAL_TIM_SetAutoreload(buzzerHtim, temp);
        __HAL_TIM_SetCompare(buzzerHtim, TIM_CHANNEL_1, temp / 2);

        /* start timer */
        HAL_TIM_PWM_Start(buzzerHtim, TIM_CHANNEL_1);
    }
    elapsedLengthBuzzer = 1;
}
