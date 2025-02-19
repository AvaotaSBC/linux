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

ESM_STATUS ESM_Kill(esm_instance_t *esm)
{
	ESM_STATUS err;

	if (esm == 0)
		return ESM_HL_NO_INSTANCE;

	ESM_FlushExceptions(esm);

	esm->esm_exception = 0;
	esm->esm_sync_lost = 0;
	esm->esm_auth_pass = 0;
	esm->esm_auth_fail = 0;

	err = esm_hostlib_mb_cmd(esm, ESM_CMD_SYSTEM_ON_EXIT_REQ, 0, 0,
					ESM_CMD_SYSTEM_ON_EXIT_RESP, 1,
					&esm->status, CMD_DEFAULT_TIMEOUT);
	/* This command is designed to cause a timeout */
	if (err != ESM_HL_COMMAND_TIMEOUT) {
		hdmi_inf("MB Failed %d", err);
		return ESM_HL_MB_FAILED;
	}

	return ESM_HL_SUCCESS;
}

ESM_STATUS ESM_Reset(esm_instance_t *esm)
{
	/* ESM_STATUS err; */

	if (esm == 0)
		return ESM_HL_NO_INSTANCE;

	ESM_FlushExceptions(esm);

	esm->esm_exception = 0;
	esm->esm_sync_lost = 0;
	esm->esm_auth_pass = 0;
	esm->esm_auth_fail = 0;

	if (esm_hostlib_mb_cmd(esm, ESM_CMD_SYSTEM_RESET_REQ, 0, 0,
		ESM_CMD_SYSTEM_RESET_RESP, 1, &esm->status, CMD_DEFAULT_TIMEOUT) != ESM_HL_SUCCESS) {
		hdmi_inf("MB Failed\n");
		return ESM_HL_MB_FAILED;
	}

	if (esm->status != ESM_SUCCESS) {
		hdmi_inf("status %d\n", esm->status);
		return ESM_HL_FAILED;
	}

	return ESM_HL_SUCCESS;
}

ESM_STATUS ESM_EnableLowValueContent(esm_instance_t *esm)
{
	ESM_STATUS err;
	uint32_t req = 0;
	uint32_t resp = 0;

	if (esm == 0)
		return ESM_HL_NO_INSTANCE;

	/* esm->fw_type = ESM_HOST_LIB_TX; */
	/* switch (esm->fw_type) {
	case ESM_HOST_LIB_TX:
		req =  ESM_HDCP_HDMI_TX_CMD_ENABLE_LOW_VALUE_CONTENT_REQ;
		resp = ESM_HDCP_HDMI_TX_CMD_ENABLE_LOW_VALUE_CONTENT_RESP;
		break;

	default:
		return ESM_HL_INVALID_COMMAND;
	} */
	req =  ESM_HDCP_HDMI_TX_CMD_ENABLE_LOW_VALUE_CONTENT_REQ;
	resp = ESM_HDCP_HDMI_TX_CMD_ENABLE_LOW_VALUE_CONTENT_RESP;
	err = esm_hostlib_mb_cmd(esm, req, 0, 0, resp, 0, &esm->status,
						CMD_DEFAULT_TIMEOUT);
	if (err != ESM_HL_SUCCESS) {
		hdmi_inf("MB Failed %d", err);
		return ESM_HL_MB_FAILED;
	}

	if (esm->status != ESM_SUCCESS) {
		hdmi_inf("status %d", esm->status);
		return ESM_HL_FAILED;
	}

	return ESM_HL_SUCCESS;
}


ESM_STATUS ESM_Authenticate(esm_instance_t *esm, uint32_t Cmd,
	uint32_t StreamID, uint32_t ContentType)
{
	uint32_t req = 0;
	uint32_t resp = 0;
	uint32_t req_param;

	if (esm == NULL) {
		hdmi_inf("[hdcp2.2-error]:esm is NULL!\n");
		return ESM_HL_NO_INSTANCE;
	}

	if (ContentType > 2) {
		hdmi_inf("[hdcp2.2-error]:content type > 2\n");
		return ESM_HL_INVALID_PARAMETERS;
	}

	ESM_FlushExceptions(esm);

	esm->esm_exception = 0;
	esm->esm_sync_lost = 0;
	esm->esm_auth_pass = 0;
	esm->esm_auth_fail = 0;
	esm->fw_type = 2;

	if (Cmd) {
		req = ESM_HDCP_HDMI_TX_CMD_AKE_START_REQ;
		resp = ESM_HDCP_HDMI_TX_CMD_AKE_START_RESP;

		req_param = ContentType;

		if (esm_hostlib_mb_cmd(esm, req, 1, &req_param, resp, 1,
			&esm->status, CMD_DEFAULT_TIMEOUT) != ESM_HL_SUCCESS) {
			hdmi_inf("[ESM_HOST_LIB_TX] MB Failed");
			return ESM_HL_MB_FAILED;
		}
	} else {
		if (esm_hostlib_mb_cmd(esm, ESM_HDCP_HDMI_TX_CMD_AKE_STOP_REQ, 0, 0,
				ESM_HDCP_HDMI_TX_CMD_AKE_STOP_RESP, 1,
				&esm->status, CMD_DEFAULT_TIMEOUT) != ESM_HL_SUCCESS) {
			hdmi_inf("MB Failed");
			return ESM_HL_MB_FAILED;
		}
	}

	if (esm->status != ESM_SUCCESS) {
		hdmi_inf("status %d", esm->status);
		return ESM_HL_FAILED;
	}
	return ESM_HL_SUCCESS;
}

ESM_STATUS ESM_SetCapability(esm_instance_t *esm)
{
	ESM_STATUS err = -1;

	if (esm == 0) {
		hdmi_inf("ESM is NULL\n");
		return ESM_HL_NO_INSTANCE;
	}

	if (esm_hostlib_mb_cmd(esm, ESM_HDCP_HDMI_TX_CMD_AKE_SET_CAPABILITY_REQ,
				0, 0, ESM_HDCP_HDMI_TX_CMD_AKE_SET_CAPABILITY_RESP, 1,
				&esm->status, CMD_DEFAULT_TIMEOUT) != ESM_HL_SUCCESS) {
		hdmi_inf("MB Failed %d", err);
		return ESM_HL_MB_FAILED;
	}

	if (esm->status != ESM_SUCCESS) {
		hdmi_inf("esm set cap status %d", esm->status);
		return ESM_HL_FAILED;
	}

	return ESM_HL_SUCCESS;
}
