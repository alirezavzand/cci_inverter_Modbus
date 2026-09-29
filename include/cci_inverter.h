#ifndef CCI_INVERTER_H
#define CCI_INVERTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CCI_INV_OK = 0,
    CCI_INV_ERR_ARGUMENT = -1,
    CCI_INV_ERR_CONNECT = -2,
    CCI_INV_ERR_IO = -3,
    CCI_INV_ERR_PROTOCOL = -4,
    CCI_INV_ERR_UNSUPPORTED = -5,
    CCI_INV_ERR_RANGE = -6,
    CCI_INV_ERR_VERIFY = -7
} cci_inv_result_t;

typedef enum {
    CCI_INV_CAP_READ_MEASUREMENTS = 1u << 0,
    CCI_INV_CAP_SET_ACTIVE_POWER  = 1u << 1,
    CCI_INV_CAP_SET_POWER_FACTOR  = 1u << 2,
    CCI_INV_CAP_SET_REACTIVE_QS   = 1u << 3
} cci_inverter_capability_t;

typedef struct {
    char vendor[24];
    char model[31];
    uint16_t model_id;
    uint16_t number_of_strings;
    uint16_t number_of_mppts;

    double rated_power_kw;
    double max_active_power_kw;
    double max_apparent_power_kva;
    double qmax_feed_to_grid_kvar;
    double qmax_absorb_from_grid_kvar;

    uint32_t capabilities;
} cci_inverter_info_t;

typedef struct {
    double dc_power_kw;

    double vab_v;
    double vbc_v;
    double vca_v;

    double ia_a;
    double ib_a;
    double ic_a;

    double active_power_kw;
    double reactive_power_kvar;
    double power_factor;
    double grid_frequency_hz;

    double efficiency_percent;
    double internal_temperature_c;

    uint16_t device_status_raw;
    uint16_t fault_code;

    uint64_t timestamp_ms;
    int valid;
} cci_inverter_measurements_t;

struct cci_inverter;
typedef struct cci_inverter cci_inverter_t;

typedef struct {
    const char *driver_name;

    int (*read_info)(void *ctx, cci_inverter_info_t *info);
    int (*read_measurements)(void *ctx, cci_inverter_measurements_t *m);

    /* Inverter-level commands. Plant-level CEI logic belongs above this API. */
    int (*set_active_power_kw)(void *ctx, double power_kw);
    int (*set_power_factor)(void *ctx, double power_factor);
    int (*set_reactive_qs)(void *ctx, double q_over_s);
} cci_inverter_ops_t;

struct cci_inverter {
    void *ctx;
    const cci_inverter_ops_t *ops;
};

int cci_inverter_read_info(cci_inverter_t *inv, cci_inverter_info_t *info);
int cci_inverter_read_measurements(
    cci_inverter_t *inv,
    cci_inverter_measurements_t *m
);
int cci_inverter_set_active_power_kw(cci_inverter_t *inv, double power_kw);
int cci_inverter_set_power_factor(cci_inverter_t *inv, double power_factor);
int cci_inverter_set_reactive_qs(cci_inverter_t *inv, double q_over_s);
const char *cci_inverter_driver_name(const cci_inverter_t *inv);

#ifdef __cplusplus
}
#endif

#endif
