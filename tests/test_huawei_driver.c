#include "cci_inverter.h"
#include "fake_modbus_transport.h"
#include "huawei_sun2000.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int nearly(double a, double b)
{
    return fabs(a - b) < 1e-6;
}

static void seed_huawei(modbus_transport_t *t)
{
    fake_modbus_set_ascii(t, 30000, 15, "SUN2000-100KTL-M2");
    fake_modbus_set_u16(t, 30070, 150);
    fake_modbus_set_u16(t, 30071, 20);
    fake_modbus_set_u16(t, 30072, 10);
    fake_modbus_set_u32(t, 30073, 100000);
    fake_modbus_set_u32(t, 30075, 110000);
    fake_modbus_set_u32(t, 30077, 110000);
    fake_modbus_set_i32(t, 30079, 66000);
    fake_modbus_set_i32(t, 30081, -66000);

    fake_modbus_set_i32(t, 32064, 88420);
    fake_modbus_set_u16(t, 32066, 3997);
    fake_modbus_set_u16(t, 32067, 4001);
    fake_modbus_set_u16(t, 32068, 3999);
    fake_modbus_set_i32(t, 32072, 125700);
    fake_modbus_set_i32(t, 32074, 125900);
    fake_modbus_set_i32(t, 32076, 125600);
    fake_modbus_set_i32(t, 32080, 87020);
    fake_modbus_set_i32(t, 32082, 0);
    fake_modbus_set_u16(t, 32084, 1000);
    fake_modbus_set_u16(t, 32085, 5001);
    fake_modbus_set_u16(t, 32086, 9840);
    fake_modbus_set_u16(t, 32087, 450);
    fake_modbus_set_u16(t, 32089, 0x0200);
    fake_modbus_set_u16(t, 32090, 0);
}

int main(void)
{
    modbus_transport_t *t = fake_modbus_create();
    huawei_sun2000_t hw;
    cci_inverter_t inv;
    cci_inverter_info_t info;
    cci_inverter_measurements_t m;
    int rc;

    assert(t != NULL);
    seed_huawei(t);

    assert(huawei_sun2000_init(&hw, t, 1, 1) == CCI_INV_OK);
    huawei_sun2000_bind(&hw, &inv);

    assert(cci_inverter_read_info(&inv, &info) == CCI_INV_OK);
    assert(strcmp(info.vendor, "Huawei") == 0);
    assert(strcmp(info.model, "SUN2000-100KTL-M2") == 0);
    assert(info.model_id == 150);
    assert(info.number_of_strings == 20);
    assert(info.number_of_mppts == 10);
    assert(nearly(info.rated_power_kw, 100.0));
    assert(nearly(info.max_active_power_kw, 110.0));
    assert(nearly(info.max_apparent_power_kva, 110.0));

    assert(cci_inverter_read_measurements(&inv, &m) == CCI_INV_OK);
    assert(nearly(m.active_power_kw, 87.020));
    assert(nearly(m.grid_frequency_hz, 50.01));
    assert(nearly(m.vab_v, 399.7));

    rc = cci_inverter_set_active_power_kw(&inv, 75.0);
    assert(rc == CCI_INV_OK);
    assert(fake_modbus_last_write_address(t) == 40126);
    assert(fake_modbus_last_write_count(t) == 2);
    assert(fake_modbus_last_write_value(t, 0) == 0x0001);
    assert(fake_modbus_last_write_value(t, 1) == 0x24F8);

    rc = cci_inverter_set_power_factor(&inv, 0.95);
    assert(rc == CCI_INV_OK);
    assert(fake_modbus_last_write_address(t) == 40122);
    assert(fake_modbus_last_write_count(t) == 1);
    assert(fake_modbus_last_write_value(t, 0) == 950);

    rc = cci_inverter_set_reactive_qs(&inv, -0.20);
    assert(rc == CCI_INV_OK);
    assert(fake_modbus_last_write_address(t) == 40123);
    assert(fake_modbus_last_write_count(t) == 1);
    assert((int16_t)fake_modbus_last_write_value(t, 0) == -200);

    assert(cci_inverter_set_active_power_kw(&inv, 120.0) == CCI_INV_ERR_RANGE);
    assert(cci_inverter_set_power_factor(&inv, 0.5) == CCI_INV_ERR_RANGE);
    assert(cci_inverter_set_reactive_qs(&inv, 1.2) == CCI_INV_ERR_RANGE);

    modbus_transport_close(t);
    printf("All Huawei driver tests passed.\n");
    return 0;
}
