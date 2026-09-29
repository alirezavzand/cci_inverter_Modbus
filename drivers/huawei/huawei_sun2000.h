#ifndef HUAWEI_SUN2000_H
#define HUAWEI_SUN2000_H

#include "cci_inverter.h"
#include "modbus_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    modbus_transport_t *transport;
    int slave_id;
    int verify_writes;

    cci_inverter_info_t cached_info;
    int info_valid;
} huawei_sun2000_t;

/* Bind one Huawei inverter on a shared Modbus RTU bus. */
int huawei_sun2000_init(
    huawei_sun2000_t *driver,
    modbus_transport_t *transport,
    int slave_id,
    int verify_writes
);

void huawei_sun2000_bind(
    huawei_sun2000_t *driver,
    cci_inverter_t *generic_inverter
);

/* Vendor-specific helpers using documented Huawei registers. */
int huawei_sun2000_read_info(
    huawei_sun2000_t *driver,
    cci_inverter_info_t *info
);

int huawei_sun2000_read_measurements(
    huawei_sun2000_t *driver,
    cci_inverter_measurements_t *m
);

/* Huawei 40126-40127, U32 W, FC16. */
int huawei_sun2000_set_active_power_kw(
    huawei_sun2000_t *driver,
    double power_kw
);

/* Huawei 40122, I16 gain 1000, FC06. */
int huawei_sun2000_set_power_factor(
    huawei_sun2000_t *driver,
    double power_factor
);

/* Huawei 40123, I16 Q/S gain 1000, FC06. */
int huawei_sun2000_set_reactive_qs(
    huawei_sun2000_t *driver,
    double q_over_s
);

#ifdef __cplusplus
}
#endif

#endif
