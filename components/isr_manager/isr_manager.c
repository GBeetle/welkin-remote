/*
 * This file is part of welkin project (https://github.com/GBeetle/welkin).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "isr_manager.h"

static uint32_t isr_counter = 0;
TaskHandle_t mpu_isr_handle;

void mpu_dmp_isr_handler(void* arg)
{
    isr_counter++;
    //ets_printf("isr before:[%s] stat:[%d] prid:[%d]\n", pcTaskGetTaskName(mpu_isr_handle), eTaskGetState(mpu_isr_handle), uxTaskPriorityGetFromISR(mpu_isr_handle));
    //xTaskResumeFromISR(mpu_isr_handle);
    //ets_printf("isr after:[%s] stat:[%d]\n", pcTaskGetTaskName(mpu_isr_handle), eTaskGetState(mpu_isr_handle));
    /* Notify the task that the transmission is complete. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_isr_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
