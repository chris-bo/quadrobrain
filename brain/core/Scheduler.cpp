/*
 * Scheduler.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#include "Scheduler.h"

Scheduler::Scheduler(Status* systemStatus, TIM_HandleTypeDef* htim,
        DiscoveryLEDs* _leds) {
    status = systemStatus;
    taskArray = 0;
    numberOfTasks = 0;
    checkedTasks = 0;
    scheduler_htim = htim;

    leds = _leds;

    /* dummy padding variable */
    pad = 0;
}

void Scheduler::start(Task** tasks, uint8_t taskAmount) {
    taskArray = tasks;
    numberOfTasks = taskAmount;
    checkedTasks = 0;
    initializeTaskDurations();

    /*  Timer set period and start */
    __HAL_TIM_SetCounter(scheduler_htim, (SCHEDULER_INTERVALL_ms * 1000));
    HAL_TIM_Base_Start_IT(scheduler_htim);
}

void Scheduler::pause() {
    /* Timer stop*/
    HAL_TIM_Base_Stop_IT(scheduler_htim);

}

void Scheduler::reset() {

    /* Timer stop*/
    HAL_TIM_Base_Stop_IT(scheduler_htim);
    /*Timer reset */
    __HAL_TIM_SetCounter(scheduler_htim, (SCHEDULER_INTERVALL_ms * 1000));
    /* reset Tasks checked */
    checkedTasks = 0;
}

void Scheduler::executeTasks() {

    if (checkedTasks == numberOfTasks) {
        /* all Tasks checked */
        checkedTasks = 0;
    } else {
        overrunError();
    }
    // Alle Tasks auf "unchecked" setzen, ausser Tasks mit Prio -1
    for (uint8_t i = 0; i < numberOfTasks; i++) {
        if (taskArray[i]->priority == -1) {
            taskArray[i]->taskChecked = true;
            checkedTasks++;
        } else {
            taskArray[i]->taskChecked = false;
        }
    }

    // Schleife läuft so lange durch, bis alle Tasks abgefragt worden sind
    for (uint8_t prio = 0; checkedTasks < numberOfTasks; prio++) {
        for (uint8_t k = 0; k < numberOfTasks; k++) {
            // Wenn Task aktuelle Priorit�t hat und aktiv (erstes Statusbit) ist
            if ((taskArray[k]->priority == prio)
                    && !(taskArray[k]->taskChecked)) {
                /* increase checkedTasks counter, and set Task as Checked*/
                checkedTasks++;
                taskArray[k]->taskChecked = true;
                /* check timer and save starting time of task*/
                uint32_t timerTmp = __HAL_TIM_GetCounter(scheduler_htim);
                /* check time
                 * */
                if ((timerTmp > taskArray[k]->duration)
                        || (taskArray[k]->priority < 2)) {
                    /* timeLeft > maxDuration or priority < 2
                     * Priority 0 and 1 are executed always
                     *
                     * */
                    taskArray[k]->update();
                    /* reset priority after task update*/
                    taskArray[k]->resetPriority();

                    /* update maxDuration */
                    timerTmp -= __HAL_TIM_GetCounter(scheduler_htim);
                    if (timerTmp > taskArray[k]->duration) {
                        /* increase duration duration */
                        taskArray[k]->duration = timerTmp;

                        /* check task durations */
                        checkTaskDurations(k);
                    } else {
                        /* timerTemp < duration -> Task needed less time
                         * decrease duration: (90% old + 10% new)
                         */
                        taskArray[k]->duration = (taskArray[k]->duration * 9
                                + timerTmp) / 10;
                    }
                } else {
                    /* not enough time to complete task
                     */

                    /* increase priority */
                    if (taskArray[k]->priority > 1) {
                        taskArray[k]->priority--;
                    }
                }

            }
        }
    }

    errorHandler();

    /* calc cpu load
     *
     * taking history into account:
     * History Factor = (CPU_LOAD_HISTORY - 1 ) / CPU_LOAD_HISTORY
     * current load = (period - idle time  )/ period
     * doesn't take this measurement into account
     *
     * newload = oldload * history factor + current load / CPU_LOAD_HISTORY
     *
     * */
    status->cpuLoad = (float) (status->cpuLoad * (CPU_LOAD_HISTORY - 1)
            + ((float) (SCHEDULER_TIMER_PERIOD
                    - __HAL_TIM_GetCounter(scheduler_htim))
                    / (float) (SCHEDULER_TIMER_PERIOD ))) / CPU_LOAD_HISTORY;

}
void Scheduler::timerIRQ() {
    /* timer interrupt -> execute Scheduler cycle */
    executeTasks();

    /* increase uptime */
    status->uptime++;
}

Scheduler::~Scheduler() {
}

void Scheduler::checkTaskDurations(uint8_t taskIndex) {
    // Summe aller Ausfuehrdauern von Aufgaben mit Prio 0
    uint32_t timeSum = 0;

    // Zeitdauern aller Tasks mit Prio 0 addieren
    for (uint8_t i = 0; i < numberOfTasks; i++) {
        if (taskArray[i]->priority == 0) {
            timeSum += taskArray[i]->duration;
        }
    }

    // Checken ob uebergebene Task selbst Prio 0 hat
    if (taskArray[taskIndex]->priority == 0) {
        // Falls ja m�ssen alle Faelle ueberprueft werden
        for (uint8_t i = 0; i < numberOfTasks; i++) {
            if (taskArray[i]->priority != 0) {
                // Wenn Prio 0 Tasks zusammen mit einer anderen Task zusammen nicht in eine Periode passen
                if ((timeSum + taskArray[i]->duration)
                        >= SCHEDULER_INTERVALL_ms * 1000) {
                    // Emergency-Flag setzen
                    status->globalFlags.emergency = true;
                    break;
                }
            }
        }
    } else {
        // Falls nicht genuegt die Addition der Ausfuehrdauer der uebergebenen Task
        timeSum += taskArray[taskIndex]->duration;

        // Wenn Ausfuehrdauer nicht in Periodendauer passt
        if (timeSum >= SCHEDULER_INTERVALL_ms * 1000) {
            // Emergency-Flag setzen
            status->globalFlags.emergency = true;
        }
    }
}

void Scheduler::errorHandler() {
    /* check cpu load */
    if (status->cpuLoad > 0.80f) {
        /* over 80% cpu load */
        leds->on(OVERLOAD_LED);
        status->globalFlags.cpuOverload = true;
        if (status->cpuLoad > 0.9f) {
            /* add error if cpu load over 90%*/
            status->globalFlags.error = true;
        }
    } else {
        /* enough cpu time left */
        leds->off(OVERLOAD_LED);
        status->globalFlags.cpuOverload = false;
    }

    /* Flight Data Reception */
    if (status->globalFlags.MPU9150ok && status->globalFlags.BMP180ok
            && status->globalFlags.RCReceiverOk
            && status->globalFlags.EEPROMok) {
        leds->on(FLIGHT_DATA_RECEPTION_LED);
    } else {
        leds->off(FLIGHT_DATA_RECEPTION_LED);
        status->globalFlags.error = true;
    }

    /* Error FLAGS */
    if (status->globalFlags.error) {
        /* some error is detected */
        /* check errors, etc */
        leds->on(ERROR_LED);
    }

    /* TODO: Scheduler: check all errorflags before
     *                  switching led off
     *
     */
    if (((status->globalFlags.lowVoltage == false)
            && (status->globalFlags.usbError == false)
            && (status->globalFlags.cpuOverload == false))
            && (status->globalFlags.MPU9150ok && status->globalFlags.BMP180ok
                    && status->globalFlags.RCReceiverOk
                    && status->globalFlags.EEPROMok)) {
        /* at this time, there are no error flags set
         * -> reset error_flag and switch led error off
         */
        status->globalFlags.error = false;
        leds->off(ERROR_LED);
    }

}

void Scheduler::initializeTaskDurations() {

    /* Disable Timerinterrupt */
    __HAL_TIM_DISABLE_IT(scheduler_htim, TIM_IT_UPDATE);

    /* Counter Mode up */
    scheduler_htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(scheduler_htim);

    for (uint8_t i = 0; i < numberOfTasks; i++) {
        /* Set Timer */
        __HAL_TIM_SetCounter(scheduler_htim, 0);
        HAL_TIM_Base_Start(scheduler_htim);

        taskArray[i]->update();
        taskArray[i]->duration = __HAL_TIM_GetCounter(scheduler_htim);

        HAL_TIM_Base_Stop(scheduler_htim);
    }

    /* Counter Mode Down */
    scheduler_htim->Init.CounterMode = TIM_COUNTERMODE_DOWN;
    HAL_TIM_Base_Init(scheduler_htim);
    __HAL_TIM_ENABLE_IT(scheduler_htim, TIM_IT_UPDATE);
}

/* kills scheduler and all tasks in taskarray */
void Scheduler::kill() {

    reset();
    for (uint8_t i = 0; i < numberOfTasks; i++) {
        taskArray[i]->kill();
    }
    /* reset Status */
    status->reset();
}

void Scheduler::overrunError() {
    /* TODO Scheduler::overrunError() */

}
