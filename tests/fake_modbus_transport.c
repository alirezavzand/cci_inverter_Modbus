#include "fake_modbus_transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct modbus_transport {
    uint16_t regs[65536];
    int last_address;
    int last_count;
    uint16_t last_values[128];
    char error[128];
};

modbus_transport_t *fake_modbus_create(void)
{
    modbus_transport_t *t = (modbus_transport_t *)calloc(1, sizeof(*t));
    if (t)
        snprintf(t->error, sizeof(t->error), "OK");
    return t;
}

modbus_transport_t *modbus_transport_open_rtu(const modbus_rtu_config_t *cfg)
{
    (void)cfg;
    return fake_modbus_create();
}

void modbus_transport_close(modbus_transport_t *t)
{
    free(t);
}

int modbus_transport_read_holding(
    modbus_transport_t *t,
    int slave_id,
    int address,
    int count,
    uint16_t *dest
)
{
    (void)slave_id;
    if (!t || !dest || address < 0 || count < 1 || address + count > 65536)
        return -1;
    memcpy(dest, &t->regs[address], (size_t)count * sizeof(uint16_t));
    return 0;
}

int modbus_transport_write_single(
    modbus_transport_t *t,
    int slave_id,
    int address,
    uint16_t value
)
{
    (void)slave_id;
    if (!t || address < 0 || address > 65535)
        return -1;
    t->regs[address] = value;
    t->last_address = address;
    t->last_count = 1;
    t->last_values[0] = value;
    return 0;
}

int modbus_transport_write_multiple(
    modbus_transport_t *t,
    int slave_id,
    int address,
    int count,
    const uint16_t *values
)
{
    (void)slave_id;
    if (!t || !values || address < 0 || count < 1 || count > 128 ||
        address + count > 65536)
        return -1;
    memcpy(&t->regs[address], values, (size_t)count * sizeof(uint16_t));
    t->last_address = address;
    t->last_count = count;
    memcpy(t->last_values, values, (size_t)count * sizeof(uint16_t));
    return 0;
}

const char *modbus_transport_last_error(modbus_transport_t *t)
{
    return t ? t->error : "no fake transport";
}

void fake_modbus_set_u16(modbus_transport_t *t, int address, uint16_t value)
{
    t->regs[address] = value;
}

void fake_modbus_set_u32(modbus_transport_t *t, int address, uint32_t value)
{
    t->regs[address] = (uint16_t)(value >> 16);
    t->regs[address + 1] = (uint16_t)(value & 0xffffU);
}

void fake_modbus_set_i32(modbus_transport_t *t, int address, int32_t value)
{
    fake_modbus_set_u32(t, address, (uint32_t)value);
}

void fake_modbus_set_ascii(modbus_transport_t *t, int address, int regs, const char *s)
{
    int i;
    size_t len = strlen(s);
    for (i = 0; i < regs; ++i) {
        size_t j = (size_t)i * 2U;
        uint16_t hi = j < len ? (uint8_t)s[j] : 0U;
        uint16_t lo = j + 1U < len ? (uint8_t)s[j + 1U] : 0U;
        t->regs[address + i] = (uint16_t)((hi << 8) | lo);
    }
}

int fake_modbus_last_write_address(modbus_transport_t *t) { return t->last_address; }
int fake_modbus_last_write_count(modbus_transport_t *t) { return t->last_count; }
uint16_t fake_modbus_last_write_value(modbus_transport_t *t, int index)
{
    if (!t || index < 0 || index >= t->last_count)
        return 0;
    return t->last_values[index];
}
