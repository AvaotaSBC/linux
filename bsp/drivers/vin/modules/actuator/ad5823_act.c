/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * sunxi actuator driver
 */
#include "../../utility/vin_log.h"
#include "actuator.h"

#define SUNXI_ACT_NAME "ad5823_act"
#define SUNXI_ACT_ID 0xd8

#define	TOTAL_STEPS_ACTIVE			64
#define USE_SINGLE_LINEAR
#define SLEWRATE 0

DEFINE_MUTEX(act_mutex_ad5823);
static struct actuator_ctrl_t act_t;

/*
 *Please implement this for each actuator device!!!
 */

static int subdev_i2c_tx(struct actuator_ctrl_t *act_ctrl, unsigned char reg,
				unsigned char val)
{
	return cci_write_a8_d8(&act_ctrl->sdev, reg, val);
}

static int subdev_i2c_write(struct actuator_ctrl_t *act_ctrl,
				unsigned short halfword, void *pt)
{
	int ret = 0;
	struct i2c_client *client;
	unsigned char vcm_data_high, vcm_data_low;

	client = act_ctrl->i2c_client;

	vcm_data_low = halfword & 0xff;
	vcm_data_high = halfword >> 8;

	ret |= subdev_i2c_tx(act_ctrl, 0x04, vcm_data_high);
	ret |= subdev_i2c_tx(act_ctrl, 0x05, vcm_data_low);

	return ret;
}

static int subdev_set_code(struct actuator_ctrl_t *act_ctrl,
				unsigned short code, unsigned short sr)
{
	int ret = 0;
	unsigned short halfword;

	halfword = code & 0x3ff;

	act_dbg("subdev_set_code[%x][%d][%d]\n", halfword, code, SLEWRATE);

	ret = subdev_i2c_write(act_ctrl, halfword, NULL);
	if (ret != 0)
		act_err("subdev set code err!\n");
	else
		act_ctrl->curr_code = code;

	return ret;
}

static int subdev_init_table(struct actuator_ctrl_t *act_ctrl,
				unsigned short ext_tbl_en,
				unsigned short ext_tbl_steps,
				unsigned short *ext_tbl)
{
	int ret = 0;
	unsigned int i;
	unsigned short *table;
	unsigned short table_size;

	act_dbg("subdev_init_table\n");

	if (ext_tbl_en == 0) {
		table_size = 2 * TOTAL_STEPS_ACTIVE * sizeof(unsigned short);
		table = (unsigned short *)kmalloc(table_size, GFP_KERNEL);

		for (i = 0; i < TOTAL_STEPS_ACTIVE; i += 1) {
			table[2 * i] = table[2 * i + 1] =
				(unsigned short)(act_ctrl->active_min +
						(unsigned int)(act_ctrl->
								active_max -
								act_ctrl->
								active_min) * i /
						(TOTAL_STEPS_ACTIVE - 1));

		}
		act_ctrl->total_steps = TOTAL_STEPS_ACTIVE;
	} else {
		table =
			(unsigned short *)kmalloc(2 * ext_tbl_steps *
						sizeof(unsigned short),
						GFP_KERNEL);
		for (i = 0; i < ext_tbl_steps; i += 1) {
			table[2 * i] = ext_tbl[2 * i];
			table[2 * i + 1] = ext_tbl[2 * i + 1];
		}
		act_ctrl->total_steps = ext_tbl_steps;
	}

	act_ctrl->step_position_table = table;
	for (i = 0; i < TOTAL_STEPS_ACTIVE; i += 1) {
		act_dbg("TBL[%d]=%d [%d]=%d ", i + 0, table[2 * i + 0], i + 1,
			table[2 * i + 1]);
	}

	return ret;
}

static int subdev_move_pos(struct actuator_ctrl_t *act_ctrl,
				unsigned short num_steps, unsigned short dir)
{
	int ret = 0;
	char sign_dir = 0;
	unsigned short target_pos = 0;
	short dest_pos = 0;
	unsigned short curr_code = 0;

	act_dbg("%s called, dir %d, num_steps %d\n", __func__, dir, num_steps);

	/* Determine sign direction */
	if (dir == MOVE_NEAR)
		sign_dir = 1;
	else if (dir == MOVE_FAR)
		sign_dir = -1;
	else {
		act_err("Illegal focus direction\n");
		ret = -EINVAL;
		return ret;
	}

	/* Determine destination step position */
	dest_pos = act_ctrl->curr_pos + (sign_dir * num_steps);

	if (dest_pos < 0)
		dest_pos = 0;
	else if (dest_pos > act_ctrl->total_steps - 1)
		dest_pos = act_ctrl->total_steps - 1;

	if (dest_pos == act_ctrl->curr_pos)
		return ret;

	act_ctrl->work_status = ACT_STA_BUSY;

	curr_code = act_ctrl->step_position_table[dir + 2 * act_ctrl->curr_pos];

	act_dbg("curr_pos =%d dest_pos =%d curr_code=%d\n",
			act_ctrl->curr_pos, dest_pos, curr_code);

	target_pos = act_ctrl->curr_pos;
	while (target_pos != dest_pos) {
		target_pos++;
		ret =
			subdev_set_code(act_ctrl,
					act_ctrl->step_position_table[dir +
									2 *
									target_pos],
					SLEWRATE);
		if (ret == 0) {
			usleep_range(1000, 1200);
			act_ctrl->curr_pos = target_pos;
		} else {
			break;
		}
	}

	act_ctrl->work_status = ACT_STA_IDLE;

	return ret;
}

static int subdev_set_pos(struct actuator_ctrl_t *act_ctrl, unsigned short pos)
{
	int ret = 0;
	unsigned short target_pos = 0;

	/* Determine destination step position */
	if (pos > act_ctrl->total_steps - 1)
		target_pos = act_ctrl->total_steps - 1;
	else
		target_pos = pos;

	act_dbg("subdev_set_pos[%d]\n", target_pos);

	if (target_pos == act_ctrl->curr_pos)
		return ret;

	ret =
		subdev_set_code(act_ctrl,
				act_ctrl->step_position_table[2 * target_pos],
				SLEWRATE);
	if (ret == 0) {
		usleep_range(1000, 1200);
		act_ctrl->curr_pos = target_pos;
	} else {
		act_err("act set pos err!");
	}

	return ret;
}

static int subdev_init(struct actuator_ctrl_t *act_ctrl,
			struct actuator_para_t *a_para)
{
	int ret = 0;
	struct actuator_para_t *para = a_para;

	if (para == NULL) {
		act_err("subdev_init para error\n");
		ret = -1;
		goto subdev_init_end;
	}

	act_dbg("act subdev_init\n");
	subdev_i2c_tx(act_ctrl, 0x02, 0x01);
	subdev_i2c_tx(act_ctrl, 0x03, 0x4B);
	act_ctrl->active_min = para->active_min;
	act_ctrl->active_max = para->active_max;

	subdev_init_table(act_ctrl, para->ext_tbl_en, para->ext_tbl_steps,
				para->ext_tbl);

	act_ctrl->curr_pos = 0;

	ret = subdev_i2c_write(act_ctrl, act_ctrl->active_min, NULL);

	if (ret == 0)
		act_ctrl->work_status = ACT_STA_IDLE;
	else
		act_ctrl->work_status = ACT_STA_ERR;

subdev_init_end:
	return ret;
}

static int subdev_pwdn(struct actuator_ctrl_t *act_ctrl, unsigned short mode)
{
	int ret = 0;

	if (mode == 1) {
		act_dbg("act subdev_pwdn %d\n", mode);
		ret = subdev_i2c_write(act_ctrl, 0, NULL);
		if (ret == 0)
			act_ctrl->work_status = ACT_STA_SOFT_PWDN;
	} else {
		act_dbg("act subdev_pwdn %d\n", mode);
		ret = 0;
		act_ctrl->work_status = ACT_STA_HW_PWDN;
	}

	return ret;
}

static int subdev_release(struct actuator_ctrl_t *act_ctrl,
				struct actuator_ctrl_word_t *ctrlwd)
{
	int ret = 0;

	act_dbg("act subdev_release to [%d], sr[%d]\n", ctrlwd->code,
		ctrlwd->sr);
	ret = subdev_i2c_write(act_ctrl, ctrlwd->code, NULL);
	if (ret == 0)
		act_ctrl->work_status = ACT_STA_IDLE;

	return ret;
}

static const struct i2c_device_id act_i2c_id[] = {
	{SUNXI_ACT_NAME, (kernel_ulong_t)&act_t},
	{}
};

static const struct v4l2_subdev_core_ops sunxi_act_subdev_core_ops = {
	.ioctl = sunxi_actuator_ioctl,
};

static struct v4l2_subdev_ops act_subdev_ops = {
	.core = &sunxi_act_subdev_core_ops,
};

static struct cci_driver cci_act_drv = {
	.type = CCI_TYPE_ACT,
	.name = SUNXI_ACT_NAME,
};

static int act_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	cci_dev_probe_helper(&act_t.sdev, client, &act_subdev_ops,
				&cci_act_drv);
	if (client)
		act_t.i2c_client = client;
	act_t.work_status = ACT_STA_HW_PWDN;
	act_dbg("%s probe\n", SUNXI_ACT_NAME);
	return 0;
}

static int act_i2c_remove(struct i2c_client *client)
{
	cci_dev_remove_helper(client, &cci_act_drv);
	return 0;
}

static struct i2c_driver act_i2c_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = SUNXI_ACT_NAME,
			},
	.id_table = act_i2c_id,
	.probe = act_i2c_probe,
	.remove = act_i2c_remove,
};

static struct actuator_ctrl_t act_t = {
	.i2c_driver = &act_i2c_driver,
	.sdev_ops = &act_subdev_ops,

	.set_info = {
			.total_steps = TOTAL_STEPS_ACTIVE,
			},

	.work_status = ACT_STA_HW_PWDN,
	.active_min = ACT_DEV_MIN_CODE,
	.active_max = ACT_DEV_MAX_CODE,

	.curr_pos = 0,
	.actuator_mutex = &act_mutex_ad5823,

	.func_tbl = {
			.actuator_init = subdev_init,
			.actuator_pwdn = subdev_pwdn,
			.actuator_i2c_write = subdev_i2c_write,
			.actuator_release = subdev_release,
			.actuator_set_code = subdev_set_code,
			.actuator_init_table = subdev_init_table,
			.actuator_move_pos = subdev_move_pos,
			.actuator_set_pos = subdev_set_pos,
			},

	.get_info = {
			.focal_length_num = 42,
			.focal_length_den = 10,
			.f_number_num = 265,
			.f_number_den = 100,
			.f_pix_num = 14,
			.f_pix_den = 10,
			.total_f_dist_num = 197681,
			.total_f_dist_den = 1000,
			},
};

static int __init act_mod_init(void)
{
	return cci_dev_init_helper(&act_i2c_driver);
}

static void __exit act_mod_exit(void)
{
	cci_dev_exit_helper(&act_i2c_driver);
}

VIN_INIT_DRIVERS(act_mod_init);
module_exit(act_mod_exit);

MODULE_DESCRIPTION("ad5823 built-in vcm actuator");
MODULE_LICENSE("GPL v2");
