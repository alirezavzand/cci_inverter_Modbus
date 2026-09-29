#include "huawei_sun2000.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/*
 * Huawei Solar Inverter Modbus Interface Definitions (V3.0), Issue 01,
 * 2023-01-17. Addresses below are the documented Huawei register addresses,
 * not simulator-only addresses.
 */
#define HUAWEI_REG_MODEL                 30000
#define HUAWEI_REG_MODEL_ID              30070
#define HUAWEI_REG_DC_POWER              32064

#define HUAWEI_REG_POWER_FACTOR_SETPOINT 40122 /* I16, gain 1000 */
#define HUAWEI_REG_REACTIVE_QS_SETPOINT  40123 /* I16, gain 1000 */
#define HUAWEI_REG_ACTIVE_POWER_FIXED    40126 /* U32 W */

#define HUAWEI_EXPECTED_MODEL_ID         150

static uint32_t decode_u32(const uint16_t *r)
{
    return ((uint32_t)r[0] << 16) | (uint32_t)r[1];
}

static int32_t decode_i32(const uint16_t *r)
{
    return (int32_t)decode_u32(r);
}

static int16_t decode_i16(uint16_t r)
{
    return (int16_t)r;
}

static void encode_u32(uint32_t value, uint16_t out[2])
{
    out[0] = (uint16_t)((value >> 16) & 0xffffU);
    out[1] = (uint16_t)(value & 0xffffU);
}

static uint64_t timestamp_ms(void)
{
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) == TIME_UTC)
        return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
    return (uint64_t)time(NULL) * 1000ULL;
}

static void decode_ascii(
    const uint16_t *regs,
    size_t register_count,
    char *dest,
    size_t dest_size
)
{
    size_t i;
    size_t out = 0;

    if (!dest || dest_size == 0)
        return;

    for (i = 0; i < register_count && out + 1 < dest_size; ++i) {
        unsigned char hi = (unsigned char)((regs[i] >> 8) & 0xff);
        unsigned char lo = (unsigned char)(regs[i] & 0xff);

        if (hi && out + 1 < dest_size)
            dest[out++] = (char)hi;
        if (lo && out + 1 < dest_size)
            dest[out++] = (char)lo;
    }

    dest[out] = '\0';
}

static int verify_registers(
    huawei_sun2000_t *driver,
    int address,
    int count,
    const uint16_t *expected
)
{
    uint16_t actual[8];
    int i;

    if (!driver->verify_writes)
        return CCI_INV_OK;

    if (count < 1 || count > (int)(sizeof(actual) / sizeof(actual[0])))
        return CCI_INV_ERR_ARGUMENT;

    if (modbus_transport_read_holding(
            driver->transport,
            driver->slave_id,
            address,
            count,
            actual) != 0)
        return CCI_INV_ERR_IO;

    for (i = 0; i < count; ++i) {
        if (actual[i] != expected[i])
            return CCI_INV_ERR_VERIFY;
    }

    return CCI_INV_OK;
}

int huawei_sun2000_init(
    huawei_sun2000_t *driver,
    modbus_transport_t *transport,
    int slave_id,
    int verify_writes
)
{
    if (!driver || !transport || slave_id < 1 || slave_id > 247)
        return CCI_INV_ERR_ARGUMENT;

    memset(driver, 0, sizeof(*driver));
    driver->transport = transport;
    driver->slave_id = slave_id;
    driver->verify_writes = verify_writes ? 1 : 0;
    return CCI_INV_OK;
}

int huawei_sun2000_read_info(
    huawei_sun2000_t *driver,
    cci_inverter_info_t *info
)
{
    uint16_t model[15];
    uint16_t r[13];

    if (!driver || !driver->transport || !info)
        return CCI_INV_ERR_ARGUMENT;

    memset(info, 0, sizeof(*info));

    if (modbus_transport_read_holding(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_MODEL,
            15,
            model) != 0)
        return CCI_INV_ERR_IO;

    if (modbus_transport_read_holding(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_MODEL_ID,
            13,
            r) != 0)
        return CCI_INV_ERR_IO;

    snprintf(info->vendor, sizeof(info->vendor), "Huawei");
    decode_ascii(model, 15, info->model, sizeof(info->model));

    info->model_id = r[0];
    info->number_of_strings = r[1];
    info->number_of_mppts = r[2];
    info->rated_power_kw = (double)decode_u32(&r[3]) / 1000.0;
    info->max_active_power_kw = (double)decode_u32(&r[5]) / 1000.0;
    info->max_apparent_power_kva = (double)decode_u32(&r[7]) / 1000.0;
    info->qmax_feed_to_grid_kvar = (double)decode_i32(&r[9]) / 1000.0;
    info->qmax_absorb_from_grid_kvar = (double)decode_i32(&r[11]) / 1000.0;

    info->capabilities =
        CCI_INV_CAP_READ_MEASUREMENTS |
        CCI_INV_CAP_SET_ACTIVE_POWER |
        CCI_INV_CAP_SET_POWER_FACTOR |
        CCI_INV_CAP_SET_REACTIVE_QS;

    driver->cached_info = *info;
    driver->info_valid = 1;

    return CCI_INV_OK;
}

int huawei_sun2000_read_measurements(
    huawei_sun2000_t *driver,
    cci_inverter_measurements_t *m
)
{
    /* 32064..32090 in one FC03 transaction. */
    uint16_t r[27];

    if (!driver || !driver->transport || !m)
        return CCI_INV_ERR_ARGUMENT;

    memset(m, 0, sizeof(*m));

    if (modbus_transport_read_holding(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_DC_POWER,
            27,
            r) != 0) {
        m->valid = 0;
        return CCI_INV_ERR_IO;
    }

    m->dc_power_kw = (double)decode_i32(&r[0]) / 1000.0;

    m->vab_v = (double)r[2] / 10.0;
    m->vbc_v = (double)r[3] / 10.0;
    m->vca_v = (double)r[4] / 10.0;

    m->ia_a = (double)decode_i32(&r[8]) / 1000.0;
    m->ib_a = (double)decode_i32(&r[10]) / 1000.0;
    m->ic_a = (double)decode_i32(&r[12]) / 1000.0;

    m->active_power_kw = (double)decode_i32(&r[16]) / 1000.0;
    m->reactive_power_kvar = (double)decode_i32(&r[18]) / 1000.0;
    m->power_factor = (double)decode_i16(r[20]) / 1000.0;
    m->grid_frequency_hz = (double)r[21] / 100.0;
    m->efficiency_percent = (double)r[22] / 100.0;
    m->internal_temperature_c = (double)decode_i16(r[23]) / 10.0;
    m->device_status_raw = r[25];
    m->fault_code = r[26];

    m->timestamp_ms = timestamp_ms();
    m->valid = 1;
    return CCI_INV_OK;
}

int huawei_sun2000_set_active_power_kw(
    huawei_sun2000_t *driver,
    double power_kw
)
{
    uint32_t raw_w;
    uint16_t regs[2];

    if (!driver || !driver->transport || !isfinite(power_kw))
        return CCI_INV_ERR_ARGUMENT;

    if (!driver->info_valid) {
        cci_inverter_info_t info;
        int rc = huawei_sun2000_read_info(driver, &info);
        if (rc != CCI_INV_OK)
            return rc;
    }

    if (power_kw < 0.0 || power_kw > driver->cached_info.max_active_power_kw)
        return CCI_INV_ERR_RANGE;

    raw_w = (uint32_t)llround(power_kw * 1000.0);
    encode_u32(raw_w, regs);

    /* U32 spans 2 holding registers: use FC16 atomically. */
    if (modbus_transport_write_multiple(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_ACTIVE_POWER_FIXED,
            2,
            regs) != 0)
        return CCI_INV_ERR_IO;

    return verify_registers(
        driver,
        HUAWEI_REG_ACTIVE_POWER_FIXED,
        2,
        regs
    );
}

int huawei_sun2000_set_power_factor(
    huawei_sun2000_t *driver,
    double power_factor
)
{
    long scaled;
    uint16_t raw;

    if (!driver || !driver->transport || !isfinite(power_factor))
        return CCI_INV_ERR_ARGUMENT;

    /*
     * The supplied SUN2000-100KTL-M2 datasheet states an adjustable range
     * from 0.8 leading to 0.8 lagging. The Huawei register is signed I16.
     * We preserve the sign supplied by the caller; interpretation of leading
     * versus lagging must be fixed by the site/Huawei convention before CEI
     * plant logic relies on it.
     */
    if (fabs(power_factor) < 0.8 || fabs(power_factor) > 1.0)
        return CCI_INV_ERR_RANGE;

    scaled = lround(power_factor * 1000.0);
    if (scaled < -32768L || scaled > 32767L)
        return CCI_INV_ERR_RANGE;

    raw = (uint16_t)(int16_t)scaled;

    if (modbus_transport_write_single(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_POWER_FACTOR_SETPOINT,
            raw) != 0)
        return CCI_INV_ERR_IO;

    return verify_registers(
        driver,
        HUAWEI_REG_POWER_FACTOR_SETPOINT,
        1,
        &raw
    );
}

int huawei_sun2000_set_reactive_qs(
    huawei_sun2000_t *driver,
    double q_over_s
)
{
    long scaled;
    uint16_t raw;

    if (!driver || !driver->transport || !isfinite(q_over_s))
        return CCI_INV_ERR_ARGUMENT;

    /* Generic safe numeric range for a signed ratio. Site-specific capability
       and Huawei mode selection will be enforced by the future control layer. */
    if (q_over_s < -1.0 || q_over_s > 1.0)
        return CCI_INV_ERR_RANGE;

    scaled = lround(q_over_s * 1000.0);
    raw = (uint16_t)(int16_t)scaled;

    if (modbus_transport_write_single(
            driver->transport,
            driver->slave_id,
            HUAWEI_REG_REACTIVE_QS_SETPOINT,
            raw) != 0)
        return CCI_INV_ERR_IO;

    return verify_registers(
        driver,
        HUAWEI_REG_REACTIVE_QS_SETPOINT,
        1,
        &raw
    );
}

static int generic_read_info(void *ctx, cci_inverter_info_t *info)
{
    return huawei_sun2000_read_info((huawei_sun2000_t *)ctx, info);
}

static int generic_read_measurements(void *ctx, cci_inverter_measurements_t *m)
{
    return huawei_sun2000_read_measurements((huawei_sun2000_t *)ctx, m);
}

static int generic_set_active_power_kw(void *ctx, double value)
{
    return huawei_sun2000_set_active_power_kw((huawei_sun2000_t *)ctx, value);
}

static int generic_set_power_factor(void *ctx, double value)
{
    return huawei_sun2000_set_power_factor((huawei_sun2000_t *)ctx, value);
}

static int generic_set_reactive_qs(void *ctx, double value)
{
    return huawei_sun2000_set_reactive_qs((huawei_sun2000_t *)ctx, value);
}

static const cci_inverter_ops_t HUAWEI_OPS = {
    "huawei_sun2000",
    generic_read_info,
    generic_read_measurements,
    generic_set_active_power_kw,
    generic_set_power_factor,
    generic_set_reactive_qs
};

void huawei_sun2000_bind(
    huawei_sun2000_t *driver,
    cci_inverter_t *generic_inverter
)
{
    if (!generic_inverter)
        return;

    generic_inverter->ctx = driver;
    generic_inverter->ops = &HUAWEI_OPS;
}
