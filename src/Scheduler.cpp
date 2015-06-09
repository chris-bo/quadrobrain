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

    maxIdleTime = 0;
    minIdleTime = 0xFFFFFFFF;
    leds = _leds;

}

void Scheduler::start(Task** tasks, uint8_t taskAmount) {
    taskArray = tasks;
    numberOfTasks = taskAmount;

    initializeTaskDurations();
    /*  Timer start */
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
    /* Timer restart */
    HAL_TIM_Base_Start_IT(scheduler_htim);
}

void Scheduler::executeTasks() {
    /* TODO scheduler: behandlung von Timer überlauf
     *
     */
    //checkedTasks zur�cksetzen
    checkedTasks = 0;
    // Alle Tasks auf "unchecked" setzen, ausser Tasks mit Prio -1
    for (uint8_t i = 0; i < numberOfTasks; i++) {
        if (taskArray[i]->priority == -1) {
            SET_FLAG(taskArray[i]->taskStatusFlags, TASK_FLAG_CHECKED);
            checkedTasks++;
        } else {
            RESET_FLAG(taskArray[i]->taskStatusFlags, TASK_FLAG_CHECKED);
        }
    }

    // Schleife läuft so lange durch, bis alle Tasks abgefragt worden sind
    for (uint8_t prio = 0; checkedTasks != numberOfTasks; prio++) {
        for (uint8_t k = 0; k < numberOfTasks; k++) {
            // Wenn Task aktuelle Priorit�t hat und aktiv (erstes Statusbit) ist
            if ((taskArray[k]->priority == prio)
                        && !(GET_FLAG(taskArray[k]->taskStatusFlags,
                                    TASK_FLAG_CHECKED))) {
                /* increase checkedTasks counter, and set Task as Checked*/
                checkedTasks++;
                SET_FLAG(taskArray[k]->taskStatusFlags, TASK_FLAG_CHECKED);
                /* check timer and save starting time of task*/
                uint32_t timerTmp = __HAL_TIM_GetCounter(scheduler_htim);
                /* check time
                 * */
                if ((timerTmp > taskArray[k]->maxDuration)
                            || (taskArray[k]->priority == 0)) {
                    /* timeLeft > maxDuration or priority == 0*/
                    taskArray[k]->update();

                    // TODO scheduler: 	maxDuration muss sich evtl wieder verringern
                    //					können, da interrupts oder breakpoints
                    //					die dauer beeinflussen -> mittelung?
                    //      		 	maxDuration -> averageDuration ?
                    //					dann muss aber ein timerüberlauf im scheduler
                    //					behandelt werden
                    /* update maxDuration */
                    if ((timerTmp - __HAL_TIM_GetCounter(scheduler_htim))
                                > taskArray[k]->maxDuration) {
                        /* update maxDuration */
                        taskArray[k]->maxDuration = timerTmp
                                    - __HAL_TIM_GetCounter(scheduler_htim);
                        /* check task durations */
                        checkTaskDurations(k);
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

    if (__HAL_TIM_GetCounter(scheduler_htim) > maxIdleTime) {
        maxIdleTime = __HAL_TIM_GetCounter(scheduler_htim);
    } else if (__HAL_TIM_GetCounter(scheduler_htim) < minIdleTime) {
        minIdleTime = __HAL_TIM_GetCounter(scheduler_htim);
    }

    errorHandler();
    /* calc cpu load
     *
     * taking history into account:
     * History Factor = (CPU_LOAD_HISTORY - 1 ) / CPU_LOAD_HISTORY
     * current load = (period - idle time - 20 ( to take this calculation into account) )/ period
     *
     *
     * newload = oldload * history factor + current load / CPU_LOAD_HISTORY
     *
     * */
    /* todo: check calculation, display cpu overload*/
    status->cpuLoad = (status->cpuLoad * (CPU_LOAD_HISTORY - 1)
                + ((float) (scheduler_htim->Init.Period
                            - __HAL_TIM_GetCounter(scheduler_htim) - 20)
                            / (float) scheduler_htim->Init.Period))
                / CPU_LOAD_HISTORY;

}
void Scheduler::timerIRQ() {
    /* timer interrupt -> execute Scheduler cycle */

    executeTasks();

}

Scheduler::~Scheduler() {

}

void Scheduler::checkTaskDurations(uint8_t taskIndex) {
    // Summe aller Ausfuehrdauern von Aufgaben mit Prio 0
    uint32_t timeSum = 0;

    // Zeitdauern aller Tasks mit Prio 0 addieren
    for (uint8_t i = 0; i < numberOfTasks; i++) {
        if (taskArray[i]->priority == 0) {
            timeSum += taskArray[i]->maxDuration;
        }
    }

    // Checken ob uebergebene Task selbst Prio 0 hat
    if (taskArray[taskIndex]->priority == 0) {
        // Falls ja m�ssen alle Faelle ueberprueft werden
        for (uint8_t i = 0; i < numberOfTasks; i++) {
            if (taskArray[i]->priority != 0) {
                // Wenn Prio 0 Tasks zusammen mit einer anderen Task zusammen nicht in eine Periode passen
                if ((timeSum + taskArray[i]->maxDuration)
                            >= SCHEDULER_INTERVALL_ms * 1000) {
                    // Emergency-Flag setzen
                    SET_FLAG(status->globalFlags, EMERGENCY_FLAG);
                    break;
                }
            }
        }
    } else {
        // Falls nicht genuegt die Addition der Ausfuehrdauer der uebergebenen Task
        timeSum += taskArray[taskIndex]->maxDuration;

        // Wenn Ausfuehrdauer nicht in Periodendauer passt
        if (timeSum >= SCHEDULER_INTERVALL_ms * 1000) {
            // Emergency-Flag setzen
            SET_FLAG(status->globalFlags, EMERGENCY_FLAG);
        }
    }
}

void Scheduler::errorHandler() {
    /* check cpu load*/
    if (status->cpuLoad > 0.75f) {
        /* over 80% cpu load */
        leds->on(OVERLOAD_LED);
        SET_FLAG(status->globalFlags, CPU_OVERLOAD_FLAG);
        if (status->cpuLoad > 0.9f) {
            SET_FLAG(status->globalFlags, ERROR_FLAG);
        }
    } else {
        leds->off(OVERLOAD_LED);
        RESET_FLAG(status->globalFlags, CPU_OVERLOAD_FLAG);
    }

    /* Error FLAGS */
    if (GET_FLAG(status->globalFlags, ERROR_FLAG)) {
        /* some error is detected */
        /* check errors, etc */

        leds->on(ERROR_LED);
    }

    /* TODO: Scheduler: ckeck all errorflags before
     *                  switching led off
     *
     */
    if (!GET_FLAG(status->globalFlags, USB_ERROR_FLAG)) {

        /* at this time, there are no set error flags
         * -> reset error_flag and switch led off
         */
        RESET_FLAG(status->globalFlags, ERROR_FLAG);
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
        taskArray[i]->maxDuration = __HAL_TIM_GetCounter(scheduler_htim);

        HAL_TIM_Base_Stop(scheduler_htim);
    }

    /* Counter Mode Down */
    scheduler_htim->Init.CounterMode = TIM_COUNTERMODE_DOWN;
    HAL_TIM_Base_Init(scheduler_htim);
    __HAL_TIM_ENABLE_IT(scheduler_htim, TIM_IT_UPDATE);
}

/* kills scheduler and all tasks in taskarray */
void Scheduler::kill() {

    /* Timer stop*/
    HAL_TIM_Base_Stop_IT(scheduler_htim);
    /*Timer reset */
    __HAL_TIM_SetCounter(scheduler_htim, (SCHEDULER_INTERVALL_ms * 1000));

    for (uint8_t i = 0; i < numberOfTasks; i++) {
        taskArray[i]->kill();
    }

}
