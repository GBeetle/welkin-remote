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

#ifndef _ISR_MANAGER__
#define _ISR_MANAGER__

#include <inttypes.h>
#include <esp_attr.h>
#include "io_define.h"

typedef enum { DATA_NOT_READY = 0, DATA_READY = 1 } data_status;

typedef struct isr_manager {
    data_status mpu_isr_status : 1;
    data_status mpu_gyro_data_status : 1;
    data_status mpu_accel_data_status : 1;
    data_status mpu_mag_data_status : 1;
    data_status reserved : 4;
} isr_manager_t;

void mpu_dmp_isr_handler(void* arg);

extern uint32_t isr_counter;
extern uint8_t rx_command_id;
extern isr_manager_t mpu_isr_manager;

#endif /* end of include guard: _TASK_MANAGER__ */
