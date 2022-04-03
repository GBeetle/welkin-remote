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

#ifndef _TASK_MANAGER__
#define _TASK_MANAGER__

#include "isr_manager.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "hal/uart_ll.h"

void uart_rx_task(void *arg);

extern struct mpu mpu;

#endif /* end of include guard: _TASK_MANAGER__ */
