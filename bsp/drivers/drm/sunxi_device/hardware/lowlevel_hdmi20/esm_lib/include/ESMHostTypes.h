/* SPDX-License-Identifier: GPL-2.0-or-later */
/*******************************************************************************
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 ******************************************************************************/
#ifndef _ESMHOSTTYPES_H_
#define _ESMHOSTTYPES_H_

#include <linux/kernel.h>
#include <linux/types.h>

#include "elliptic_system_types.h"

#define ESM_STATUS ELP_STATUS

/**
 * details
 * This structure contains the ESM's last internal status when queried.
 */
typedef struct {
	uint32_t esm_exception;
	uint32_t esm_sync_lost;/* /< Indicates that the synchronization lost. */
	uint32_t esm_auth_pass;/* /< Indicates that the last AKE Start command was passed. */
	uint32_t esm_auth_fail;/* /< Indicates that the last AKE Start command has failed. */
} esm_status_t;

/**
 * \details
 *
 * This structure will be filled when ESM firmware successfully started and it contains
 * ESM buffers configuration values.
 *
 */
typedef struct {
	uint32_t esm_type;               /* /< Indicates what ESM firmware running: 0-unknown; 1-RX; 2-TX. */
	uint32_t topo_buffer_size;       /* /< Indicates maximum size of a topology slot memory. */
	uint8_t  topo_slots;             /* /< Indicates amount of topology slot memories. */
	uint8_t  topo_seed_buffer_size;  /* /< Indicates maximum size of the topology seed memory. */
	uint32_t log_buffer_size;        /* /< Indicates maximum size of the logging memory. */
	uint32_t mb_buffer_size;         /* /< Indicates maximum size of the mailbox memory. */
	uint32_t exceptions_buffer_size; /* /< Indicates maximum size of the exceptions memory. */
	uint32_t srm_buffer_size;        /* /< Indicates maximum size of the TX SRM memory. */
	uint32_t pairing_buffer_size;    /* /< Indicates maximum size of the TX Pairing memory. */
} esm_config_t;

#endif

