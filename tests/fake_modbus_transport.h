#ifndef FAKE_MODBUS_TRANSPORT_H
#define FAKE_MODBUS_TRANSPORT_H

#include "modbus_transport.h"
#include <stdint.h>

modbus_transport_t *fake_modbus_create(void);
void fake_modbus_set_u16(modbus_transport_t *t, int address, uint16_t value);
void fake_modbus_set_u32(modbus_transport_t *t, int address, uint32_t value);
void fake_modbus_set_i32(modbus_transport_t *t, int address, int32_t value);
void fake_modbus_set_ascii(modbus_transport_t *t, int address, int regs, const char *s);
int fake_modbus_last_write_address(modbus_transport_t *t);
int fake_modbus_last_write_count(modbus_transport_t *t);
uint16_t fake_modbus_last_write_value(modbus_transport_t *t, int index);

#endif
