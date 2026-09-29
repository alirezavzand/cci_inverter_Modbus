#include "modbus_transport.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <modbus.h>

#ifdef _WIN32
#include <ctype.h>
#endif

struct modbus_transport {
    modbus_t *ctx;
    char last_error[256];
};

static void set_error(modbus_transport_t *t, const char *prefix)
{
    if (!t)
        return;
    snprintf(
        t->last_error,
        sizeof(t->last_error),
        "%s: %s",
        prefix ? prefix : "libmodbus",
        modbus_strerror(errno)
    );
}

static int validate_slave(int slave_id)
{
    return slave_id >= 1 && slave_id <= 247;
}

#ifdef _WIN32
static const char *normalize_windows_port(
    const char *input,
    char *buffer,
    size_t buffer_size
)
{
    int number = 0;

    if (!input || !buffer || buffer_size < 8)
        return input;

    if (strncmp(input, "\\\\.\\", 4) == 0)
        return input;

    if ((input[0] == 'C' || input[0] == 'c') &&
        (input[1] == 'O' || input[1] == 'o') &&
        (input[2] == 'M' || input[2] == 'm')) {
        const char *p = input + 3;
        if (*p) {
            while (*p && isdigit((unsigned char)*p)) {
                number = number * 10 + (*p - '0');
                ++p;
            }
            if (*p == '\0' && number >= 10) {
                snprintf(buffer, buffer_size, "\\\\.\\%s", input);
                return buffer;
            }
        }
    }

    return input;
}
#endif

modbus_transport_t *modbus_transport_open_rtu(const modbus_rtu_config_t *cfg)
{
    modbus_transport_t *t;
    const char *port;
#ifdef _WIN32
    char normalized_port[64];
#endif
    uint32_t timeout_sec;
    uint32_t timeout_usec;

    if (!cfg || !cfg->serial_port || cfg->baud_rate <= 0 ||
        (cfg->parity != 'N' && cfg->parity != 'E' && cfg->parity != 'O') ||
        cfg->data_bits < 5 || cfg->data_bits > 8 ||
        (cfg->stop_bits != 1 && cfg->stop_bits != 2)) {
        return NULL;
    }

    t = (modbus_transport_t *)calloc(1, sizeof(*t));
    if (!t)
        return NULL;

#ifdef _WIN32
    port = normalize_windows_port(
        cfg->serial_port,
        normalized_port,
        sizeof(normalized_port)
    );
#else
    port = cfg->serial_port;
#endif

    t->ctx = modbus_new_rtu(
        port,
        cfg->baud_rate,
        cfg->parity,
        cfg->data_bits,
        cfg->stop_bits
    );

    if (!t->ctx) {
        snprintf(t->last_error, sizeof(t->last_error), "modbus_new_rtu failed");
        free(t);
        return NULL;
    }

    timeout_sec = (uint32_t)(cfg->response_timeout_ms > 0
        ? cfg->response_timeout_ms / 1000
        : 1);
    timeout_usec = (uint32_t)(cfg->response_timeout_ms > 0
        ? (cfg->response_timeout_ms % 1000) * 1000
        : 0);

    if (timeout_sec == 0 && timeout_usec == 0)
        timeout_usec = 1000000U - 1U;

    if (modbus_set_response_timeout(t->ctx, timeout_sec, timeout_usec) == -1) {
        set_error(t, "modbus_set_response_timeout");
        modbus_free(t->ctx);
        free(t);
        return NULL;
    }

    if (cfg->byte_timeout_ms > 0) {
        uint32_t byte_sec = (uint32_t)(cfg->byte_timeout_ms / 1000);
        uint32_t byte_usec = (uint32_t)((cfg->byte_timeout_ms % 1000) * 1000);
        if (modbus_set_byte_timeout(t->ctx, byte_sec, byte_usec) == -1) {
            set_error(t, "modbus_set_byte_timeout");
            modbus_free(t->ctx);
            free(t);
            return NULL;
        }
    } else {
        (void)modbus_set_byte_timeout(t->ctx, 0, 0);
    }

    modbus_set_debug(t->ctx, cfg->debug ? 1 : 0);

    if (modbus_connect(t->ctx) == -1) {
        set_error(t, "modbus_connect");
        modbus_free(t->ctx);
        free(t);
        return NULL;
    }

#if defined(__linux__) && defined(MODBUS_RTU_RS485)
    if (cfg->request_kernel_rs485) {
        if (modbus_rtu_set_serial_mode(t->ctx, MODBUS_RTU_RS485) == -1) {
            set_error(t, "modbus_rtu_set_serial_mode(RS485)");
            modbus_close(t->ctx);
            modbus_free(t->ctx);
            free(t);
            return NULL;
        }
    }
#else
    (void)cfg->request_kernel_rs485;
#endif

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return t;
}

void modbus_transport_close(modbus_transport_t *t)
{
    if (!t)
        return;
    if (t->ctx) {
        modbus_close(t->ctx);
        modbus_free(t->ctx);
    }
    free(t);
}

static int select_slave(modbus_transport_t *t, int slave_id)
{
    if (!t || !t->ctx || !validate_slave(slave_id)) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid Modbus slave ID");
        return -1;
    }

    if (modbus_set_slave(t->ctx, slave_id) == -1) {
        set_error(t, "modbus_set_slave");
        return -1;
    }
    return 0;
}

int modbus_transport_read_holding(
    modbus_transport_t *t,
    int slave_id,
    int address,
    int count,
    uint16_t *dest
)
{
    int rc;

    if (!t || !dest || address < 0 || address > 65535 ||
        count < 1 || count > 125 || address + count - 1 > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC03 arguments");
        return -1;
    }

    if (select_slave(t, slave_id) != 0)
        return -1;

    rc = modbus_read_registers(t->ctx, address, count, dest);
    if (rc != count) {
        if (rc == -1)
            set_error(t, "modbus_read_registers");
        else
            snprintf(t->last_error, sizeof(t->last_error),
                     "Short FC03 response: expected %d registers, got %d", count, rc);
        return -1;
    }

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return 0;
}

int modbus_transport_write_single(
    modbus_transport_t *t,
    int slave_id,
    int address,
    uint16_t value
)
{
    int rc;

    if (!t || address < 0 || address > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC06 arguments");
        return -1;
    }

    if (select_slave(t, slave_id) != 0)
        return -1;

    rc = modbus_write_register(t->ctx, address, value);
    if (rc != 1) {
        if (rc == -1)
            set_error(t, "modbus_write_register");
        else
            snprintf(t->last_error, sizeof(t->last_error),
                     "Unexpected FC06 result: %d", rc);
        return -1;
    }

    snprintf(t->last_error, sizeof(t->last_error), "OK");
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
    int rc;

    if (!t || !values || address < 0 || address > 65535 ||
        count < 1 || count > 123 || address + count - 1 > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC16 arguments");
        return -1;
    }

    if (select_slave(t, slave_id) != 0)
        return -1;

    rc = modbus_write_registers(t->ctx, address, count, values);
    if (rc != count) {
        if (rc == -1)
            set_error(t, "modbus_write_registers");
        else
            snprintf(t->last_error, sizeof(t->last_error),
                     "Short FC16 write: expected %d registers, got %d", count, rc);
        return -1;
    }

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return 0;
}

const char *modbus_transport_last_error(modbus_transport_t *t)
{
    return t ? t->last_error : "No Modbus transport";
}
