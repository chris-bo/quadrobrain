/*
 * PPMGenerator.cpp
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#include "PPMGenerator.h"

PPMGenerator::PPMGenerator(Status* statusPtr, uint8_t defaultPrio,
        TIM_HandleTypeDef* htim, float* _controllerValueX,
        float* _controllerValueY, float* _controllerValueZ, float* _throttle) :
        Task(statusPtr, defaultPrio) {
    PPMGenerator_htim = htim;
    controllerValueX = _controllerValueX;
    controllerValueY = _controllerValueY;
    controllerValueZ = _controllerValueZ;
    throttle = _throttle;

    counter = 0;
    motorValuesAvg[0] = 0;
    motorValuesAvg[1] = 0;
    motorValuesAvg[2] = 0;
    motorValuesAvg[3] = 0;

    /* dummy padding variable */
    pad = 0;
}

PPMGenerator::~PPMGenerator() {

}

void PPMGenerator::update() {
    /* motor 2 + 4 : X-Axis
     * motor 1 + 3 : Y-Axis
     */

    // Motoren aktiviert?
    if (status->rcSignalEnable) {
        // Throttle hoch genug, damit Regler aktiv werden darf?

        /* todo ppmgen: test sign of controllervalue Z*/
        if (*throttle > PPM_GENERATOR_THROTTLE_THRESHOLD) {
            motorValuesAvg[0] += *throttle * THROTTLE_SCALING
                    - *controllerValueY + *controllerValueZ;
            motorValuesAvg[1] += *throttle * THROTTLE_SCALING
                    - *controllerValueX - *controllerValueZ;
            motorValuesAvg[2] += *throttle * THROTTLE_SCALING
                    + *controllerValueY + *controllerValueZ;
            motorValuesAvg[3] += *throttle * THROTTLE_SCALING
                    + *controllerValueX - *controllerValueZ;
            counter++;
        } else {
            // Throttle zu niedrig => Reglerwerte werden nicht beruecksichtig
            motorValuesAvg[0] = *throttle;
            motorValuesAvg[1] = *throttle;
            motorValuesAvg[2] = *throttle;
            motorValuesAvg[3] = *throttle;
            /* keine Mittelwertsbildung -> wert direkt uebernehmen */
            counter = 1;
        }

    } else {
        // Motoren sind deaktiviert => alle Werte auf 0 setzen
        motorValuesAvg[0] = 0;
        motorValuesAvg[1] = 0;
        motorValuesAvg[2] = 0;
        motorValuesAvg[3] = 0;
        /* keine Mittelwertsbildung -> wert direkt uebernehmen */
        counter = 1;
    }

    if (__HAL_TIM_GetCounter(PPMGenerator_htim)
            >= PPM_TIMER_PERIOD * (20 - SCHEDULER_INTERVALL_ms) / 20) {
        /* Im letzten Scheduler Intervall vor der neuen PWM Periode werden die neuen
         * Werte gesetzt
         */
        status->motorValues[0] = motorValuesAvg[0] / counter;
        status->motorValues[1] = motorValuesAvg[1] / counter;
        status->motorValues[2] = motorValuesAvg[2] / counter;
        status->motorValues[3] = motorValuesAvg[3] / counter;

        /* reset counter and avg*/
        counter = 0;
        motorValuesAvg[0] = 0;
        motorValuesAvg[1] = 0;
        motorValuesAvg[2] = 0;
        motorValuesAvg[3] = 0;

        // Fuer alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte fuer den Timer wandeln
        // und anschliessend dem Timer uebergeben
        // Motor 1
        uint16_t temp = (uint16_t) (status->motorValues[0]
                * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
                + PPM_TIMER_MIN_PULSE_LENGTH);
        __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
        // Motor 2
        temp = (uint16_t) (status->motorValues[1]
                * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
                + PPM_TIMER_MIN_PULSE_LENGTH);
        __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
        // Motor 3
        temp = (uint16_t) (status->motorValues[2]
                * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
                + PPM_TIMER_MIN_PULSE_LENGTH);
        __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
        // Motor 4
        temp = (uint16_t) (status->motorValues[3]
                * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
                + PPM_TIMER_MIN_PULSE_LENGTH);
        __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_4, temp);

    }
}

void PPMGenerator::initialize() {
    // PPM-Timer starten
    HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_4);

    // Motor-Kalibrierung

    // F�r alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte f�r den Timer wandeln
    // und anschlie�end dem Timer �bergeben
    // Motor 1
    uint16_t temp = (uint16_t) (status->motorValues[0]
            * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
            + PPM_TIMER_MIN_PULSE_LENGTH);
    __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
    // Motor 2
    temp = (uint16_t) (status->motorValues[1]
            * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
            + PPM_TIMER_MIN_PULSE_LENGTH);
    __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
    // Motor 3
    temp = (uint16_t) (status->motorValues[2]
            * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
            + PPM_TIMER_MIN_PULSE_LENGTH);
    __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
    // Motor 4
    temp = (uint16_t) (status->motorValues[3]
            * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH)
            + PPM_TIMER_MIN_PULSE_LENGTH);
    __HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_4, temp);

    // Testen ob sich Motoren korrekt Kalibrieren
    for (uint32_t i = 0; i < PPM_SETUP_TIME; i++) {
        __asm__("nop");
    }

    taskActive = true;
}

void PPMGenerator::disableMotors() {

    // PPM-Timer stoppen
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_4);

}

void PPMGenerator::kill() {
    disableMotors();
    reset();
    taskActive = false;
}
