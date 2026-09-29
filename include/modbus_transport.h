#ifndef MODBUS_TRANSPORT_H
#define MODBUS_TRANSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct modbus_transport modbus_transport_t;

typedef struct {
    const char *serial_port;  /* Windows: COM2; OpenWrt: /dev/rs485_2_uart */
    int baud_rate;
    char parity;              /* 'N', 'E', 'O' */
    int data_bits;            /* normally 8 */
    int stop_bits;            /* 1 or 2 */
    int response_timeout_ms;
    int byte_timeout_ms;      /* 0 disables byte timeout */
    int request_kernel_rs485; /* Linux only; default 0 */
    int debug;                /* libmodbus frame debug */
} modbus_rtu_config_t;

modbus_transport_t *modbus_transport_open_rtu(const modbus_rtu_config_t *cfg);
void modbus_transport_close(modbus_transport_t *transport);

/* Slave ID is supplied per transaction so one serial bus can host many inverters. */
int modbus_transport_read_holding(
    modbus_transport_t *transport,
    int slave_id,
    int address,
    int count,
    uint16_t *dest
);

/* FC06 */
int modbus_transport_write_single(
    modbus_transport_t *transport,
    int slave_id,
    int address,
    uint16_t value
);

/* FC16 */
int modbus_transport_write_multiple(
    modbus_transport_t *transport,
    int slave_id,
    int address,
    int count,
    const uint16_t *values
);

const char *modbus_transport_last_error(modbus_transport_t *transport);

#ifdef __cplusplus
}
#endif

#endif
