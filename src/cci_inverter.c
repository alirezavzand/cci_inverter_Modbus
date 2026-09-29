#include "cci_inverter.h"

static int validate_base(const cci_inverter_t *inv)
{
    if (!inv || !inv->ops || !inv->ctx)
        return CCI_INV_ERR_ARGUMENT;
    return CCI_INV_OK;
}

int cci_inverter_read_info(cci_inverter_t *inv, cci_inverter_info_t *info)
{
    int rc = validate_base(inv);
    if (rc != CCI_INV_OK || !info)
        return CCI_INV_ERR_ARGUMENT;
    if (!inv->ops->read_info)
        return CCI_INV_ERR_UNSUPPORTED;
    return inv->ops->read_info(inv->ctx, info);
}

int cci_inverter_read_measurements(
    cci_inverter_t *inv,
    cci_inverter_measurements_t *m
)
{
    int rc = validate_base(inv);
    if (rc != CCI_INV_OK || !m)
        return CCI_INV_ERR_ARGUMENT;
    if (!inv->ops->read_measurements)
        return CCI_INV_ERR_UNSUPPORTED;
    return inv->ops->read_measurements(inv->ctx, m);
}

int cci_inverter_set_active_power_kw(cci_inverter_t *inv, double power_kw)
{
    int rc = validate_base(inv);
    if (rc != CCI_INV_OK)
        return rc;
    if (!inv->ops->set_active_power_kw)
        return CCI_INV_ERR_UNSUPPORTED;
    return inv->ops->set_active_power_kw(inv->ctx, power_kw);
}

int cci_inverter_set_power_factor(cci_inverter_t *inv, double power_factor)
{
    int rc = validate_base(inv);
    if (rc != CCI_INV_OK)
        return rc;
    if (!inv->ops->set_power_factor)
        return CCI_INV_ERR_UNSUPPORTED;
    return inv->ops->set_power_factor(inv->ctx, power_factor);
}

int cci_inverter_set_reactive_qs(cci_inverter_t *inv, double q_over_s)
{
    int rc = validate_base(inv);
    if (rc != CCI_INV_OK)
        return rc;
    if (!inv->ops->set_reactive_qs)
        return CCI_INV_ERR_UNSUPPORTED;
    return inv->ops->set_reactive_qs(inv->ctx, q_over_s);
}

const char *cci_inverter_driver_name(const cci_inverter_t *inv)
{
    if (!inv || !inv->ops || !inv->ops->driver_name)
        return "unknown";
    return inv->ops->driver_name;
}
