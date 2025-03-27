// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for MPS2856
 * Monolithic Power Systems VR Controllers
 * - only supports rail 1!
 *
 * Copyright (C) 2023 Quanta Computer lnc.
 * Copyright (C) 2022, 2025 Antevia Networks Ltd.
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pmbus.h>
#include "pmbus.h"

/* Vendor specific registers. */
#define MP2856_MFR_VR_CONFIG2       0x5e
#define MP2856_VOUT_MODE        BIT(11)

#define MP2856_PAGE_NUM             1        // only support rail 1

#define MP2856_REGISTER_PAGE_0      0
#define MP2856_REGISTER_PAGE_2      2

#define MP2856_MUL1_CUR_SCALE_R1    0x0b
#define MP2856_PWR_EXPONENT_BIT_SET (0x1f << 6)    // set power exponent to -1

#define MP2856_MUL1_IIN_CONFIG      0x0f
#define MP2856_MUL1_PMBUS_ADDR_SET  0x1a

enum chips { mp2856, mp2857 };

static const struct i2c_device_id mp2856_id[] = {
    {"mp2856", mp2856},
    {}
};

MODULE_DEVICE_TABLE(i2c, mp2856_id);

struct mp2856_data {
    struct pmbus_driver_info info;

    int vout_format;
};

#define to_mp2856_data(x)    container_of(x, struct mp2856_data, info)

static int
mp2856_raw_read_word(struct i2c_client *client, int page, u8 reg, u16 mask)
{
    int ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, page);

    if (ret < 0)
        return ret;

    ret = i2c_smbus_read_word_data(client, reg);

    return (ret > 0) ? ret & mask : ret;
}


static int
mp2856_read_word_helper(struct i2c_client *client,
        int page, int phase, u8 reg, u16 mask)
{
    int ret = pmbus_read_word_data(client, page, phase, reg);

    return (ret > 0) ? ret & mask : ret;
}

static int
mp2856_read_vout(struct i2c_client *client, struct mp2856_data *data, int page,
         int phase, u8 reg)
{
    int ret;

    ret = mp2856_read_word_helper(client, page, phase, reg, GENMASK(9, 0));
    if (ret < 0)
        return ret;

    /* convert vout result to direct format */
    ret = (data->vout_format == vid) ? ((ret + 49) * 5) : ((ret * 1000) >> 8);

    return ret;
}


static int
mp2856_read_word_data(struct i2c_client *client, int page,
              int phase, int reg)
{
    const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
    struct mp2856_data *data = to_mp2856_data(info);
    int ret = ~EINVAL;

    switch (reg) {

//    case PMBUS_OT_FAULT_LIMIT:        // 4Fh
//    case PMBUS_OT_FAULT_RESPONSE:     // 50h
//    case PMBUS_OT_WARN_LIMIT:         // 51h

//    case PMBUS_VIN_OV_FAULT_LIMIT:    // 55h

    // these registers are supported directly by the MP2856

    case PMBUS_STATUS_BYTE:         // 78h
    case PMBUS_STATUS_WORD:         // 79h
    case PMBUS_STATUS_VOUT:         // 7Ah
    case PMBUS_STATUS_IOUT:         // 7Bh
    case PMBUS_STATUS_INPUT:        // 7Ch
    case PMBUS_STATUS_CML:          // 7Eh
    case PMBUS_READ_VIN:            // 88h
    case PMBUS_READ_TEMPERATURE_1:  // 8Dh,
    case PMBUS_READ_POUT:           // 96h

        ret = pmbus_read_word_data(client, page, phase, reg);
        break;

    case PMBUS_READ_VOUT: //8Bh
        ret = mp2856_read_vout(client, data, page, phase, reg);
        break;

    case PMBUS_READ_IOUT: //8Ch
        if (phase == 0xff)
            ret = pmbus_read_word_data(client, page, phase, reg);

        break;

    default:
        ret = -EINVAL;    // read is not possible on the reg address given
//        return -ENODATA; // returning this will cause a regular read
        break;

    }

    return ret;
}

static int
mp2856_read_byte_data(struct i2c_client *client, int page, int reg)
{
    switch (reg) {
    case PMBUS_VOUT_MODE:
        /* Enforce VOUT direct format. */
        return PB_VOUT_MODE_DIRECT;
    default:
        return -ENODATA;
    }
}


static int
mp2856_set_pout_exponent(struct i2c_client *client,
                  struct mp2856_data *data)
{
    int ret;
    u16 reg_data;

    ret = mp2856_raw_read_word(client,
            MP2856_REGISTER_PAGE_2, MP2856_MUL1_CUR_SCALE_R1, ~(u16)(GENMASK(10,6)));

    if (ret < 0)
        return ret;

    reg_data = ret | MP2856_PWR_EXPONENT_BIT_SET;

    ret = i2c_smbus_write_word_data(client, MP2856_MUL1_CUR_SCALE_R1, reg_data);

    if (ret < 0)
        return ret;

    return 0;
}

static int
mp2856_identify_vout_format(struct i2c_client *client, struct mp2856_data *data)
{
    int ret;

    ret = mp2856_raw_read_word(client,
            MP2856_REGISTER_PAGE_0, MP2856_MFR_VR_CONFIG2, ~0);

    if (ret < 0)
        return ret;

    data->vout_format = (ret & MP2856_VOUT_MODE) ? linear : vid;

    return 0;
}

static struct pmbus_driver_info mp2856_info = {
    .pages = MP2856_PAGE_NUM,
    .format[PSC_VOLTAGE_IN] = linear,
    .format[PSC_VOLTAGE_OUT] = direct,
    .format[PSC_CURRENT_OUT] = linear,
    .format[PSC_POWER] = linear,
    .m[PSC_VOLTAGE_OUT] = 1,
    .R[PSC_VOLTAGE_OUT] = 3,
    .func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
         PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
         PMBUS_HAVE_POUT | PMBUS_HAVE_STATUS_INPUT,
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

    /* rail2 is not active */
    info->pages = 1;

    /* Obtain input current sense gain of power stage. */
    ret = mp2856_set_pout_exponent(client, data);
    if (ret < 0)
        return ret;

    /* Identify vout format. */
    ret = mp2856_identify_vout_format(client, data);
    if (ret < 0)
        return ret;

    /* set the device to page 0 */
    i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);

    return pmbus_do_probe(client, info);
}

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
MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_AUTHOR("Robert Butt <Robert.B@antevianetworks.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MP2856 device, rail1");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("PMBUS");
