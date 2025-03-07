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
#include "include/ESMHost.h"

/* Puts an entry into the FIFO */
void esm_hostlib_put_exception(esm_instance_t *esm, uint32_t c)
{
	int i = (esm->exp_status_buffer.head + 1) % EX_BUFFER_SIZE;

	if (i != esm->exp_status_buffer.tail) {
		esm->exp_status_buffer.buffer[esm->exp_status_buffer.head] = c;
		esm->exp_status_buffer.head = i;
	}
}

/* Puts esm firmware exceptions from external memory into the FIFO */
ESM_STATUS esm_hostlib_put_exceptions(esm_instance_t *esm)
{
	struct ESM_EV {
		uint32_t i;/* id */
		uint32_t f;/* exception_flag */
		uint32_t v;/* exception_value */
		uint32_t a;/* action */
		uint32_t l;/* file_line */
		char n[16];
	};

	uint8_t exceptions_buffer[sizeof(struct ESM_EV)];

	struct ESM_EV *ev = (struct ESM_EV *)exceptions_buffer;
	uint32_t rsz = esm->esm_exceptions_size;
	uint32_t exc_offset = 0;
	uint32_t last_id = 0;
	uint32_t last_id_put = 0;

	/* copy data exceptions buffer */
	if (esm->driver->data_read(esm->driver->instance, esm->esm_exc_off,
			exceptions_buffer, sizeof(struct ESM_EV)) != ESM_HL_SUCCESS) {
		hdmi_err("ESM_EV data read failed\n");
		return -1;
	}

	if (!ev->i)
		/* Empty */
		return ESM_HL_SUCCESS;

	/* Locate the last exception we put in the status fifo */
	/* If the exception table was wrapped up already */
	if ((esm->esm_exceptions_last_id) && (ev->i > esm->esm_exceptions_last_id)) {
		while (rsz > 0) {
			if (ev->i == esm->esm_exceptions_last_id) {
				rsz -= sizeof(struct ESM_EV);

				exc_offset += sizeof(struct ESM_EV);
				if (esm->driver->data_read(esm->driver->instance, (esm->esm_exc_off + exc_offset),
						exceptions_buffer, sizeof(struct ESM_EV)) != ESM_HL_SUCCESS)
					return -1;
				ev = (struct ESM_EV *)exceptions_buffer;
				break;
			}

			rsz -= sizeof(struct ESM_EV);

			exc_offset += sizeof(struct ESM_EV);
			if (esm->driver->data_read(esm->driver->instance, (esm->esm_exc_off + exc_offset),
					exceptions_buffer, sizeof(struct ESM_EV)) != ESM_HL_SUCCESS)
				return -1;
			ev = (struct ESM_EV *)exceptions_buffer;
		}
	}

	/* Apparently the last one was the last in the table */
	/* and latest is on the top start from the top */
	if (rsz == 0) {
		exc_offset = 0;
		if (esm->driver->data_read(esm->driver->instance, (esm->esm_exc_off + exc_offset),
				exceptions_buffer, sizeof(struct ESM_EV)) != ESM_HL_SUCCESS)
			return -1;
		ev = (struct ESM_EV *)exceptions_buffer;
		rsz = esm->esm_exceptions_size;
	}

	while (rsz > 0) {
		if (!ev->i)
			break;

		last_id = ev->i;

		/* Only NOTIFY is pushed to the STATUS */
		if (last_id > esm->esm_exceptions_last_id) {
			last_id_put = last_id;
			/* only exception_flag is put into FIFO */
			esm_hostlib_put_exception(esm, ev->f);
		}

		rsz -= sizeof(struct ESM_EV);

		exc_offset += sizeof(struct ESM_EV);
		if (esm->driver->data_read(esm->driver->instance, (esm->esm_exc_off + exc_offset),
				exceptions_buffer, sizeof(struct ESM_EV)) != ESM_HL_SUCCESS)
			return -1;
		ev = (struct ESM_EV *)exceptions_buffer;

	}

	if (last_id_put)
		esm->esm_exceptions_last_id = last_id_put;

	return ESM_HL_SUCCESS;
}

/* Flushes all NOTIFY exceptions in the history exceptions FIFO */
void ESM_FlushExceptions(esm_instance_t *esm)
{
	esm->exp_status_buffer.head = esm->exp_status_buffer.tail;
}

