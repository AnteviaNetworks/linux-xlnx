// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for MPS Multi-phase Digital VR Controllers
 *
 * Based on mp2856.c
 * Copyright (C) 2020 Nvidia Technologies Ltd.
 * Copyright (C) 2022 Antevia Networks Ltd.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

/* Vendor specific registers. */
#define MP2856_MUL1_SYS_CONFIG2_R1	0x0d
#define MP2856_MUL1_SYS_CONFIG2_R2	0x1d
#define MP2856_MUL1_SYS_CONFIG3_R1	0x0e
#define MP2856_MUL1_SYS_CONFIG3_R2	0x1e
#define MP2856_MFR_VR_CONFIG2 		0x5e
#define MP2856_MFR_VR_CONFIG1		0x68
#define MP2856_MFR_READ_CS1_2		0x82
#define MP2856_MFR_READ_CS3_4		0x83
#define MP2856_MFR_READ_CS5_6		0x84
#define MP2856_MFR_READ_CS7_8		0x85
#define MP2856_MFR_READ_CS9_10		0x86
#define MP2856_MFR_READ_CS11_12		0x87
#define MP2856_MUL1_CUR_SCALE_R1	0x0b
#define MP2856_MUL1_CUR_SCALE_R2	0x1b
#define MP2856_MFR_READ_IOUT_PK		0x90
#define MP2856_MFR_READ_POUT_PK		0x91
#define MP2856_MUL1_SYS_CONFIG1_R1	0x03
#define MP2856_MUL1_SYS_CONFIG1_R2	0x13
#define MP2856_SYS_CONFIG1_R1		0x03
#define MP2856_SYS_CONFIG1_R2		0x13
#define MP2856_MFR_UVP_SET		0xe6

#define MP2856_VOUT_FORMAT		BIT(11)
#define MP2856_PRT_THRES_DIV_OV_EN	BIT(14)
#define MP2856_DRMOS_KCS		GENMASK(13, 12)
#define MP2856_PROT_DEV_OV_OFF          10
#define MP2856_PROT_DEV_OV_ON           5
#define MP2856_SENSE_AMPL		BIT(9)
#define MP2856_SENSE_AMPL_UNIT		1
#define MP2856_SENSE_AMPL_HALF		2
#define MP2856_VIN_UV_LIMIT_UNIT	8

#define MP2856_MAX_PHASE_RAIL1	8
#define MP2856_MAX_PHASE_RAIL2	4
#define MP2856_PAGE_NUM		2

#define MP2856_RAIL2_FUNC	(PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT | \
				 PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | \
				 PMBUS_HAVE_POUT | PMBUS_PHASE_VIRTUAL)

struct mp2856_data {
	struct pmbus_driver_info info;
	int vout_scale[MP2856_PAGE_NUM];
	int iout_scale[MP2856_PAGE_NUM];
	int pout_exp[MP2856_PAGE_NUM];
	int vid_step[MP2856_PAGE_NUM];
	int vref[MP2856_PAGE_NUM];
	int vref_off[MP2856_PAGE_NUM];
	int vout_max[MP2856_PAGE_NUM];
	int vout_ov_fixed[MP2856_PAGE_NUM];
	int vout_format[MP2856_PAGE_NUM];
	int curr_sense_gain[MP2856_PAGE_NUM];
};

#define to_mp2856_data(x)  container_of(x, struct mp2856_data, info)

static int mp2856_read_byte_data(struct i2c_client *client, int page, int reg)
{
	switch (reg) {
	case PMBUS_VOUT_MODE:
		/*
		 * Enforce VOUT direct format, since device allows to set the
		 * different formats for the different rails. Conversion from
		 * VID to direct provided by driver internally, in case it is
		 * necessary.
		 */
		return PB_VOUT_MODE_DIRECT;
	default:
		return -ENODATA;
	}
}

static int
mp2856_read_word_helper(struct i2c_client *client, int page, int phase, u8 reg,
			u16 mask)
{
	int ret = pmbus_read_word_data(client, page, phase, reg);

	return (ret > 0) ? ret & mask : ret;
}

static int
mp2856_read_phase(struct i2c_client *client, struct mp2856_data *data,
		  int page, int phase, u8 reg)
{
	int ph_curr, ret;

	ret = pmbus_read_word_data(client, page, phase, reg);
	if (ret < 0)
		return ret;

	if (!((phase + 1) % MP2856_PAGE_NUM))
		ret >>= 8;
	ret &= 0xff;

	/*
	 * Output value is calculated as: (READ_CSx / 80 – 1.23) / (Kcs * Rcs)
	 * where:
	 * - Kcs is the DrMOS current sense gain of power stage, which is
	 *   obtained from the register MP2856_MFR_VR_CONFIG1, bits 13-12 with
	 *   the following selection of DrMOS (data->curr_sense_gain[page]):
	 *   00b - 5µA/A, 01b - 8.5µA/A, 10b - 9.7µA/A, 11b - 10µA/A.
	 * - Rcs is the internal phase current sense resistor which is constant
	 *   value 1kΩ.
	 */
	ph_curr = ret * 100 - 9800;

	/*
	 * Current phase sensing, providing by the device is not accurate
	 * for the light load. This because sampling of current occurrence of
	 * bit weight has a big deviation for light load. For handling such
	 * case phase current is represented as the maximum between the value
	 * calculated  above and total rail current divided by number phases.
	 */
	ret = pmbus_read_word_data(client, page, phase, PMBUS_READ_IOUT);
	if (ret < 0)
		return ret;

	return max_t(int, DIV_ROUND_CLOSEST(ret, data->info.phases[page]),
		     DIV_ROUND_CLOSEST(ph_curr, data->curr_sense_gain[page]));
}

static int
mp2856_read_phases(struct i2c_client *client, struct mp2856_data *data,
		   int page, int phase)
{
	int ret;

	if (page) {
		switch (phase) {
		case 0 ... 1:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS7_8);
			break;
		case 2 ... 3:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS9_10);
			break;
		case 4 ... 5:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS11_12);
			break;
		default:
			return -ENODATA;
		}
	} else {
		switch (phase) {
		case 0 ... 1:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS1_2);
			break;
		case 2 ... 3:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS3_4);
			break;
		case 4 ... 5:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS5_6);
			break;
		case 6 ... 7:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS7_8);
			break;
		case 8 ... 9:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS9_10);
			break;
		case 10 ... 11:
			ret = mp2856_read_phase(client, data, page, phase,
						MP2856_MFR_READ_CS11_12);
			break;
		default:
			return -ENODATA;
		}
	}
	return ret;
}

static int
mp2856_over_voltage_protection_get(struct i2c_client *client, int page)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Get over volyage protection thresholde for rail 1 or rail 2. */
	if(page == 0)
		ret = i2c_smbus_read_word_data(client, MP2856_MUL1_SYS_CONFIG1_R1);
	else
		ret = i2c_smbus_read_word_data(client, MP2856_MUL1_SYS_CONFIG1_R2);
	if (ret < 0)
		return ret;

	/*
	 * OVP Delta Voltage = OVP_DELTA x 50mV + 50mV
	 */
	return (ret & GENMASK(12,10)) >> 10;
}

static int mp2856_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct mp2856_data *data = to_mp2856_data(info);
	int ret;
	u16 exponent, mantissa;

	switch (reg) {
	case PMBUS_OT_FAULT_LIMIT:
		ret = mp2856_read_word_helper(client, page, phase, reg,
					      GENMASK(7, 0));
		break;
	case PMBUS_VIN_OV_FAULT_LIMIT:
		ret = mp2856_read_word_helper(client, page, phase, reg,
					      GENMASK(7, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(ret, MP2856_VIN_UV_LIMIT_UNIT);
		break;
	case PMBUS_VOUT_OV_FAULT_LIMIT:
		/*
		 * Register provides two values for over-voltage protection
		 * threshold for fixed (ovp2) and tracking (ovp1) modes. The
		 * minimum of these two values is provided as over-voltage
		 * fault alarm.
		 * OVP Delta Voltage = OVP_DELTA x 50mV + 50mV
		 */
		ret = mp2856_over_voltage_protection_get(client, page);

		if (ret < 0)
			return ret;

		ret = min_t(int, data->vout_max[page] + 50 * (ret + 1),
			    data->vout_ov_fixed[page]);
		break;
	case PMBUS_VOUT_UV_FAULT_LIMIT:
		ret = mp2856_read_word_helper(client, page, phase,
					      MP2856_MFR_UVP_SET,
					      GENMASK(2, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(data->vref[page] * 10 - 50 *
					(ret + 1) * data->vout_scale[page], 10);
		break;
	case PMBUS_READ_VOUT:
		ret = mp2856_read_word_helper(client, page, phase, reg,
					      GENMASK(11, 0));
		if (ret < 0)
			return ret;

		/*
		 * READ_VOUT can be provided in VID or direct format. The
		 * format type is specified by bit 11 of the register
		 * MP2856_MFR_VR_CONFIG2. The driver enforces VOUT direct
		 * format, since device allows to set the different formats for
		 * the different rails and also all VOUT limits registers are
		 * provided in a direct format. In case format is VID - convert
		 * to direct.
		 */
		ret = DIV_ROUND_CLOSEST(ret * data->vid_step[page], 100000);
		break;
	case PMBUS_VIRT_READ_POUT_MAX:
		ret = mp2856_read_word_helper(client, page, phase,
					      MP2856_MFR_READ_POUT_PK,
					      GENMASK(15, 0));
		if (ret < 0)
			return ret;

                exponent = ((s16)ret) >> 11;
                mantissa = ((s16)((ret & GENMASK(10, 0)) << 5)) >> 5;
                if (exponent >= 0)
                        ret = mantissa << exponent;
                else
                        ret = mantissa >> -exponent;

                if (data->pout_exp[page] >= 0)
                        ret = ret << data->pout_exp[page];
                else
                        ret = ret >> -data->pout_exp[page];
		break;
	case PMBUS_VIRT_READ_IOUT_MAX:
		ret = mp2856_read_word_helper(client, page, phase,
					      MP2856_MFR_READ_IOUT_PK,
					      GENMASK(15, 0));
		if (ret < 0)
			return ret;

                exponent = ((s16)ret) >> 11;
                mantissa = ((s16)((ret & GENMASK(10, 0)) << 5)) >> 5;
                if (exponent >= 0)
                        ret = mantissa << exponent;
                else
                        ret = mantissa >> -exponent;

		if(data->iout_scale[page] > 0)
			ret = DIV_ROUND_CLOSEST(ret, -data->iout_scale[page]);
		if(data->iout_scale > 0)
			ret *= data->iout_scale[page];
		break;
	case PMBUS_READ_IOUT:
		ret = mp2856_read_phases(client, data, page, phase);
		if (ret < 0)
			return ret;

		break;
	case PMBUS_UT_WARN_LIMIT:
	case PMBUS_UT_FAULT_LIMIT:
	case PMBUS_VIN_UV_WARN_LIMIT:
	case PMBUS_VIN_UV_FAULT_LIMIT:
	case PMBUS_VOUT_UV_WARN_LIMIT:
	case PMBUS_VOUT_OV_WARN_LIMIT:
	case PMBUS_VIN_OV_WARN_LIMIT:
	case PMBUS_IIN_OC_FAULT_LIMIT:
	case PMBUS_IOUT_OC_LV_FAULT_LIMIT:
	case PMBUS_IIN_OC_WARN_LIMIT:
	case PMBUS_IOUT_OC_WARN_LIMIT:
	case PMBUS_IOUT_OC_FAULT_LIMIT:
	case PMBUS_IOUT_UC_FAULT_LIMIT:
	case PMBUS_POUT_OP_FAULT_LIMIT:
	case PMBUS_POUT_OP_WARN_LIMIT:
	case PMBUS_PIN_OP_WARN_LIMIT:
		return -ENXIO;
	default:
		return -ENODATA;
	}

	return ret;
}

static int mp2856_identify_multiphase_rail2(struct i2c_client *client)
{
	int ret;

	/*
	 * Identify multiphase for rail 2 - could be from 0 to 4.
	 * In case phase number is zero – only page zero is supported
	 */
	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify multiphase for rail 2 - could be from 0 to 4. */
	ret = i2c_smbus_read_word_data(client, MP2856_MUL1_SYS_CONFIG2_R2);
	if (ret < 0)
		return ret;

	ret &= GENMASK(2, 0);
	return (ret >= 4) ? 4 : ret;
}

static void mp2856_set_phase_rail1(struct pmbus_driver_info *info)
{
	int i;

	for (i = 0 ; i < info->phases[0]; i++)
		info->pfunc[i] = PMBUS_HAVE_IOUT;
}

static void
mp2856_set_phase_rail2(struct pmbus_driver_info *info, int num_phases)
{
	int i;

	/* Set phases for rail 2 from upper to lower. */
	for (i = 1; i <= num_phases; i++)
		info->pfunc[MP2856_MAX_PHASE_RAIL1 - i] = PMBUS_HAVE_IOUT;
}

static int
mp2856_identify_multiphase(struct i2c_client *client, struct mp2856_data *data,
			   struct pmbus_driver_info *info)
{
	int num_phases2, ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify multiphase for rail 1 - could be from 1 to 8. */
	ret = i2c_smbus_read_word_data(client, MP2856_MUL1_SYS_CONFIG2_R1);
	if (ret <= 0)
		return ret;

	info->phases[0] = ret & GENMASK(3, 0);

	/*
	 * The device provides a total of 8 PWM pins, and can be configured
	 * to different phase count applications for rail 1 and rail 2.
	 * Rail 1 can be set to 8 phases, while rail 2 can only be set to 4
	 * phases at most. When rail 1’s phase count is configured as 0, rail
	 * 1 operates with 1-phase DCM. When rail 2 phase count is configured
	 * as 0, rail 2 is disabled.
	 */
	if (info->phases[0] > MP2856_MAX_PHASE_RAIL1)
		return -EINVAL;

	mp2856_set_phase_rail1(info);
	num_phases2 = min(MP2856_MAX_PHASE_RAIL1 - info->phases[0],
			  MP2856_MAX_PHASE_RAIL2);
	if (info->phases[1] && info->phases[1] <= num_phases2)
		mp2856_set_phase_rail2(info, num_phases2);

	return 0;
}

static int
mp2856_identify_vid(struct i2c_client *client, struct mp2856_data *data,
		    struct pmbus_driver_info *info, int page)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, page);
	if (ret < 0)
		return ret;

        /* Identify VID mode and step selection. */
        ret = i2c_smbus_read_word_data(client, MP2856_MFR_VR_CONFIG2);
        if (ret < 0)
                return ret;

	// vid_step units = mV * 100000
        if (ret & MP2856_VOUT_FORMAT) {
                data->vout_format[page] = linear;
		data->vid_step[page] = 390625;
	}
        else
	{
                data->vout_format[page] = vid;
		data->vid_step[page] = 500000;
	}
	info->vrm_version[page] = amd625mv;
	return 0;
}

static int
mp2856_identify_rails_vid(struct i2c_client *client, struct mp2856_data *data,
			  struct pmbus_driver_info *info)
{
	int ret;

	/* Identify VID mode for rail 1. */
	ret = mp2856_identify_vid(client, data, info, 0);
	if (ret < 0)
		return ret;

	/* Identify VID mode for rail 2, if connected. */
	if (info->phases[1])
		ret = mp2856_identify_vid(client, data, info, 1);
	return ret;
}

static int
mp2856_current_sense_gain_get(struct i2c_client *client,
			      struct mp2856_data *data)
{
	int i, ret;

	/*
	 * Obtain DrMOS current sense gain of power stage from the register
	 * MP2856_MFR_VR_CONFIG1, bits 13-12. The value is selected as below:
	 * 00b - 5µA/A, 01b - 8.5µA/A, 10b - 9.7µA/A, 11b - 10µA/A. Other
	 * values are invalid.
	 */
	for (i = 0 ; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;
		ret = i2c_smbus_read_word_data(client,
					       MP2856_MFR_VR_CONFIG1);
		if (ret < 0)
			return ret;

		switch ((ret & MP2856_DRMOS_KCS) >> 12) {
		case 0:
			data->curr_sense_gain[i] = 50;
			break;
		case 1:
			data->curr_sense_gain[i] = 85;
			break;
		case 2:
			data->curr_sense_gain[i] = 97;
			break;
		default:
			data->curr_sense_gain[i] = 100;
			break;
		}
	}

	return 0;
}

static int
mp2856_vref_get(struct i2c_client *client, struct mp2856_data *data,
		struct pmbus_driver_info *info)
{
	int i, ret, vout_scale, vout;

	for (i = 0 ; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;
		ret = i2c_smbus_read_word_data(client, PMBUS_VOUT_SCALE_LOOP);
		if (ret < 0)
			return ret;

		vout_scale = ret & GENMASK(8, 0);

		ret = i2c_smbus_read_word_data(client, PMBUS_READ_VOUT);
		if (ret < 0)
			return ret;

		vout = ret & GENMASK(9, 0);

		switch ((ret & MP2856_DRMOS_KCS) >> 12) {
		case 0:
			data->curr_sense_gain[i] = 50;
			break;
		case 1:
			data->curr_sense_gain[i] = 85;
			break;
		case 2:
			data->curr_sense_gain[i] = 97;
			break;
		default:
			data->curr_sense_gain[i] = 100;
			break;
		}
	}
        /* Select the gain of remote sense amplifier. */
        ret = i2c_smbus_read_word_data(client, PMBUS_VOUT_SCALE_LOOP);
        if (ret < 0)
                return ret;

	vout_scale = ret & GENMASK(8, 0);

	return 0;
}

static int
mp2856_vref_offset_get(struct i2c_client *client, struct mp2856_data *data,
		       int page)
{
	int ret;
	int reg =  page ? MP2856_SYS_CONFIG1_R2 : MP2856_SYS_CONFIG1_R1;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		return ret;

	if(ret)
		data->vref_off[page] = (ret & GENMASK(12, 10) >> 10) * 50 + 50;
	else 
		data->vref_off[page] = 0;
	return 0;
}

static int
mp2856_vout_max_get(struct i2c_client *client, struct mp2856_data *data,
		    struct pmbus_driver_info *info, int page)
{
	int ret;

	/* Get maximum reference voltage of VID-DAC in VID format. */
	ret = i2c_smbus_read_word_data(client, PMBUS_VOUT_MAX);
	if (ret < 0)
		return ret;

	data->vout_max[page] = (ret & GENMASK(8, 0)) * 20;
	return 0;
}

static int
mp2856_vout_ov_scale_get(struct i2c_client *client, struct mp2856_data *data,
			 struct pmbus_driver_info *info)
{
	int i, thres_dev, sense_ampl, ret;
	u8 config_reg;

	for (i = 0, config_reg = MP2856_MUL1_SYS_CONFIG3_R1;
		i < data->info.pages;
		i++, config_reg = MP2856_MUL1_SYS_CONFIG3_R2) {

		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
		if (ret < 0)
			return ret;

		/*
		 * Get divider for over- and under-voltage protection thresholds
		 * configuration from the Advanced Options of Auto Phase Shedding and
		 * decay register.
		 */
		ret = i2c_smbus_read_word_data(client, config_reg);
		if (ret < 0)
			return ret;
		thres_dev = ret & MP2856_PRT_THRES_DIV_OV_EN ? MP2856_PROT_DEV_OV_ON :
							       MP2856_PROT_DEV_OV_OFF;

		sense_ampl = ret & MP2856_SENSE_AMPL ? MP2856_SENSE_AMPL_HALF :
						       MP2856_SENSE_AMPL_UNIT;

		data->vout_scale[i] = sense_ampl * thres_dev;
	}

	return 0;
}

static int
mp2856_iout_pout_scale_get(struct i2c_client *client, struct mp2856_data *data,
			 struct pmbus_driver_info *info)
{
	int i, iout_scale, pout_exp, ret;
	u8 config_reg;

	for (i = 0, config_reg = MP2856_MUL1_CUR_SCALE_R1;
		i < data->info.pages;
		i++, config_reg = MP2856_MUL1_CUR_SCALE_R2) {

		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
		if (ret < 0)
			return ret;

		/*
		 * Get divider for over- and under-voltage protection thresholds
		 * configuration from the Advanced Options of Auto Phase Shedding and
		 * decay register.
		 */
		ret = i2c_smbus_read_word_data(client, config_reg);
		if (ret < 0)
			return ret;
		iout_scale = ret & GENMASK(2, 0);
		switch(iout_scale) {
		case 0:
			data->iout_scale[i] = 1; // 1A per LSB
			break;
		case 1:
			data->iout_scale[i] = -32; // 1/32A per LSB
			break;
		case 2:
			data->iout_scale[i] = -18; // 1/16A per LSB
			break;
		case 3:
			data->iout_scale[i] = -8; // 1/8A per LSB
			break;
		case 4:
			data->iout_scale[i] = -4; // 1/4A per LSB
			break;
		case 5:
			data->iout_scale[i] = -2; // 1/2A per LSB
			break;
		case 6:
			data->iout_scale[i] = 1; // 1A per LSB
			break;
		case 7:
			data->iout_scale[i] = 2; // 2A per LSB
			break;
		}

		pout_exp = ret & GENMASK(10,6);
		pout_exp <<= (sizeof(pout_exp) * BITS_PER_BYTE - 10 -1);
		pout_exp >>= (sizeof(pout_exp) * BITS_PER_BYTE - (10 - 6 + 1));
		data->pout_exp[i] = pout_exp;
	}

	return 0;
}

static int
mp2856_vout_per_rail_config_get(struct i2c_client *client,
				struct mp2856_data *data,
				struct pmbus_driver_info *info)
{
	int i, ret;

	for (i = 0; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;

		/* Obtain voltage reference offsets. */
		ret = mp2856_vref_offset_get(client, data, i);
		if (ret < 0)
			return ret;

		/* Obtain maximum voltage values. */
		ret = mp2856_vout_max_get(client, data, info, i);
		if (ret < 0)
			return ret;

		/*
		 * Set over-voltage fixed value. Thresholds are provided as
		 * fixed value, and tracking value. The minimum of them are
		 * exposed as over-voltage critical threshold.
		 */
		data->vout_ov_fixed[i] = data->vref[i] +
					 DIV_ROUND_CLOSEST(data->vref_off[i] *
							   data->vout_scale[i],
							   10);
	}

	return 0;
}

static struct pmbus_driver_info mp2856_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.m[PSC_TEMPERATURE] = 1,
	.m[PSC_VOLTAGE_OUT] = 1,
	.R[PSC_VOLTAGE_OUT] = 3,
	.m[PSC_CURRENT_OUT] = 1,
	.m[PSC_POWER] = 1,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_POUT |
		PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT | PMBUS_PHASE_VIRTUAL,
	.read_byte_data = mp2856_read_byte_data,
	.read_word_data = mp2856_read_word_data,
};

static int mp2856_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	struct mp2856_data *data;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(struct mp2856_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memcpy(&data->info, &mp2856_info, sizeof(*info));
	info = &data->info;

	/* Identify multiphase configuration for rail 2. */
	ret = mp2856_identify_multiphase_rail2(client);
	if (ret < 0)
		return ret;

	if (ret) {
		/* Two rails are connected. */
		data->info.pages = MP2856_PAGE_NUM;
		data->info.phases[1] = ret;
		data->info.func[1] = MP2856_RAIL2_FUNC;
	}

	/* Identify multiphase configuration. */
	ret = mp2856_identify_multiphase(client, data, info);
	if (ret)
		return ret;

	/* Identify VID setting per rail. */
	ret = mp2856_identify_rails_vid(client, data, info);
	if (ret < 0)
		return ret;

	/* Obtain current sense gain of power stage. */
	ret = mp2856_current_sense_gain_get(client, data);
	if (ret)
		return ret;

	/* Obtain voltage reference values. */
	ret = mp2856_vref_get(client, data, info);
	if (ret)
		return ret;

	/* Obtain vout over-voltage scales. */
	ret = mp2856_vout_ov_scale_get(client, data, info);
	if (ret < 0)
		return ret;

	/* Obtain current out scales and power out exponents. */
	ret = mp2856_iout_pout_scale_get(client, data, info);
	if (ret < 0)
		return ret;

	/* Obtain offsets, maximum and format for vout. */
	ret = mp2856_vout_per_rail_config_get(client, data, info);
	if (ret)
		return ret;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id mp2856_id[] = {
	{"mp2856", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mp2856_id);

static const struct of_device_id __maybe_unused mp2856_of_match[] = {
	{.compatible = "mps,mp2856"},
	{}
};
MODULE_DEVICE_TABLE(of, mp2856_of_match);

static struct i2c_driver mp2856_driver = {
	.driver = {
		.name = "mp2856",
		.of_match_table = of_match_ptr(mp2856_of_match),
	},
	.probe_new = mp2856_probe,
	.id_table = mp2856_id,
};

module_i2c_driver(mp2856_driver);

MODULE_AUTHOR("Steve Williams <Steve.W@antevianetworks.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MP2856 device");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
