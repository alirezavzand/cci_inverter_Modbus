#include "modbus_transport.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct modbus_transport {
    HANDLE handle;
    int response_timeout_ms;
    int byte_timeout_ms;
    int debug;
    char last_error[256];
};

static uint16_t modbus_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffffU;
    size_t pos;
    int bit;

    for (pos = 0; pos < length; ++pos) {
        crc ^= (uint16_t)data[pos];
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 1U)
                crc = (uint16_t)((crc >> 1) ^ 0xa001U);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }

    return crc;
}

static int valid_slave(int slave_id)
{
    return slave_id >= 1 && slave_id <= 247;
}

static const char *windows_port_name(
    const char *input,
    char *buffer,
    size_t buffer_size
)
{
    if (!input || !buffer || buffer_size < 8)
        return input;

    if (strncmp(input, "\\\\.\\", 4) == 0)
        return input;

    if ((input[0] == 'C' || input[0] == 'c') &&
        (input[1] == 'O' || input[1] == 'o') &&
        (input[2] == 'M' || input[2] == 'm')) {
        snprintf(buffer, buffer_size, "\\\\.\\%s", input);
        return buffer;
    }

    return input;
}

static void debug_bytes(const char *prefix, const uint8_t *data, size_t length)
{
    size_t i;

    if (!prefix || !data)
        return;

    fputs(prefix, stdout);
    for (i = 0; i < length; ++i)
        printf("%s%.2X%s", i == 0 ? "[" : "[", data[i], "]");
    putchar('\n');
}

static int set_comm_timeouts(modbus_transport_t *t)
{
    COMMTIMEOUTS timeouts;
    DWORD first_timeout;

    if (!t)
        return -1;

    memset(&timeouts, 0, sizeof(timeouts));

    /*
     * ReadFile() is used synchronously. Keep each call short so the outer
     * response timeout remains authoritative. Once the first byte arrives,
     * read_exact() still enforces the total transaction timeout.
     */
    first_timeout = (DWORD)(t->byte_timeout_ms > 0 ? t->byte_timeout_ms : 20);
    if (first_timeout < 1)
        first_timeout = 1;

    timeouts.ReadIntervalTimeout = first_timeout;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = first_timeout;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1000;

    if (!SetCommTimeouts(t->handle, &timeouts)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "SetCommTimeouts failed: Win32 error %lu",
            (unsigned long)GetLastError()
        );
        return -1;
    }

    return 0;
}

static int read_exact(
    modbus_transport_t *t,
    uint8_t *buffer,
    size_t wanted
)
{
    size_t got = 0;
    ULONGLONG started;

    if (!t || !buffer)
        return -1;

    started = GetTickCount64();

    while (got < wanted) {
        DWORD nread = 0;
        ULONGLONG elapsed = GetTickCount64() - started;

        if (elapsed >= (ULONGLONG)t->response_timeout_ms) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "Modbus RTU timeout (%u/%u bytes)",
                (unsigned)got,
                (unsigned)wanted
            );
            return -1;
        }

        if (!ReadFile(
                t->handle,
                buffer + got,
                (DWORD)(wanted - got),
                &nread,
                NULL)) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "ReadFile failed: Win32 error %lu",
                (unsigned long)GetLastError()
            );
            return -1;
        }

        if (nread == 0) {
            Sleep(1);
            continue;
        }

        got += (size_t)nread;
    }

    return 0;
}

static int write_frame(
    modbus_transport_t *t,
    const uint8_t *frame,
    size_t length
)
{
    DWORD written = 0;

    if (!t || !frame || length == 0)
        return -1;

    if (!PurgeComm(t->handle, PURGE_RXCLEAR | PURGE_RXABORT)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "PurgeComm failed: Win32 error %lu",
            (unsigned long)GetLastError()
        );
        return -1;
    }

    if (t->debug)
        debug_bytes("TX ", frame, length);

    if (!WriteFile(t->handle, frame, (DWORD)length, &written, NULL)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "WriteFile failed: Win32 error %lu",
            (unsigned long)GetLastError()
        );
        return -1;
    }

    if ((size_t)written != length) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Short serial write: expected %u bytes, wrote %lu",
            (unsigned)length,
            (unsigned long)written
        );
        return -1;
    }

    /*
     * For a physical Windows COM port, FlushFileBuffers() waits until queued
     * transmit data has been sent.  Some virtual null-modem drivers (used by
     * our PC simulator setup) do not implement this operation and return
     * ERROR_INVALID_FUNCTION / ERROR_NOT_SUPPORTED even though WriteFile()
     * succeeded and the RTU frame was delivered to the peer.
     *
     * Treat only those two driver-capability errors as non-fatal.  Any other
     * FlushFileBuffers failure still aborts the transaction.  OpenWrt does not
     * use this backend; it uses the libmodbus transport.
     */
    if (!FlushFileBuffers(t->handle)) {
        DWORD err = GetLastError();

        if (err == ERROR_INVALID_FUNCTION || err == ERROR_NOT_SUPPORTED) {
            if (t->debug) {
                printf(
                    "NOTE FlushFileBuffers unsupported by this COM driver "
                    "(Win32 error %lu); continuing after successful WriteFile.\n",
                    (unsigned long)err
                );
            }
        }
        else {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "FlushFileBuffers failed: Win32 error %lu",
                (unsigned long)err
            );
            return -1;
        }
    }

    return 0;
}

static int validate_crc(modbus_transport_t *t, const uint8_t *frame, size_t length)
{
    uint16_t calculated;
    uint16_t received;

    if (!t || !frame || length < 4)
        return -1;

    calculated = modbus_crc16(frame, length - 2U);
    received = (uint16_t)frame[length - 2U] |
               ((uint16_t)frame[length - 1U] << 8);

    if (calculated != received) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "CRC mismatch: received 0x%04X, calculated 0x%04X",
            (unsigned)received,
            (unsigned)calculated
        );
        return -1;
    }

    return 0;
}

static int read_response_prefix(
    modbus_transport_t *t,
    int slave_id,
    uint8_t expected_function,
    uint8_t prefix[3]
)
{
    if (read_exact(t, prefix, 3) != 0)
        return -1;

    if (prefix[0] != (uint8_t)slave_id) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected slave ID %u (expected %d)",
            (unsigned)prefix[0],
            slave_id
        );
        return -1;
    }

    if (prefix[1] == (uint8_t)(expected_function | 0x80U)) {
        uint8_t crc_tail[2];
        uint8_t exception_frame[5];

        if (read_exact(t, crc_tail, 2) != 0)
            return -1;

        exception_frame[0] = prefix[0];
        exception_frame[1] = prefix[1];
        exception_frame[2] = prefix[2];
        exception_frame[3] = crc_tail[0];
        exception_frame[4] = crc_tail[1];

        if (t->debug)
            debug_bytes("RX ", exception_frame, sizeof(exception_frame));

        if (validate_crc(t, exception_frame, sizeof(exception_frame)) != 0)
            return -1;

        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Modbus exception code %u",
            (unsigned)prefix[2]
        );
        return -1;
    }

    if (prefix[1] != expected_function) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected function 0x%02X (expected 0x%02X)",
            (unsigned)prefix[1],
            (unsigned)expected_function
        );
        return -1;
    }

    return 0;
}

modbus_transport_t *modbus_transport_open_rtu(const modbus_rtu_config_t *cfg)
{
    modbus_transport_t *t;
    DCB dcb;
    char normalized[64];
    const char *port;

    if (!cfg || !cfg->serial_port || cfg->baud_rate <= 0 ||
        (cfg->parity != 'N' && cfg->parity != 'E' && cfg->parity != 'O') ||
        cfg->data_bits < 5 || cfg->data_bits > 8 ||
        (cfg->stop_bits != 1 && cfg->stop_bits != 2)) {
        return NULL;
    }

    t = (modbus_transport_t *)calloc(1, sizeof(*t));
    if (!t)
        return NULL;

    t->handle = INVALID_HANDLE_VALUE;
    t->response_timeout_ms = cfg->response_timeout_ms > 0
        ? cfg->response_timeout_ms
        : 1000;
    t->byte_timeout_ms = cfg->byte_timeout_ms;
    t->debug = cfg->debug ? 1 : 0;

    port = windows_port_name(cfg->serial_port, normalized, sizeof(normalized));

    if (t->debug)
        printf("Opening %s at %d bauds (%c, %d, %d)\n",
               port, cfg->baud_rate, cfg->parity, cfg->data_bits, cfg->stop_bits);

    t->handle = CreateFileA(
        port,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (t->handle == INVALID_HANDLE_VALUE) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "CreateFile(%s) failed: Win32 error %lu",
            port,
            (unsigned long)GetLastError()
        );
        free(t);
        return NULL;
    }

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(t->handle, &dcb)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    dcb.BaudRate = (DWORD)cfg->baud_rate;
    dcb.ByteSize = (BYTE)cfg->data_bits;
    dcb.StopBits = cfg->stop_bits == 2 ? TWOSTOPBITS : ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;

    if (cfg->parity == 'E') {
        dcb.Parity = EVENPARITY;
        dcb.fParity = TRUE;
    } else if (cfg->parity == 'O') {
        dcb.Parity = ODDPARITY;
        dcb.fParity = TRUE;
    } else {
        dcb.Parity = NOPARITY;
        dcb.fParity = FALSE;
    }

    if (!SetCommState(t->handle, &dcb)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    if (!SetupComm(t->handle, 4096, 4096)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    if (set_comm_timeouts(t) != 0) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    (void)PurgeComm(
        t->handle,
        PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT
    );

    (void)cfg->request_kernel_rs485; /* Linux-only option */

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return t;
}

void modbus_transport_close(modbus_transport_t *t)
{
    if (!t)
        return;

    if (t->handle != INVALID_HANDLE_VALUE)
        CloseHandle(t->handle);

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
    uint8_t request[8];
    uint8_t prefix[3];
    uint8_t frame[260];
    uint16_t crc;
    size_t total;
    int i;

    if (!t || !dest || !valid_slave(slave_id) ||
        address < 0 || address > 65535 ||
        count < 1 || count > 125 || address + count - 1 > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC03 arguments");
        return -1;
    }

    request[0] = (uint8_t)slave_id;
    request[1] = 0x03;
    request[2] = (uint8_t)((address >> 8) & 0xff);
    request[3] = (uint8_t)(address & 0xff);
    request[4] = (uint8_t)((count >> 8) & 0xff);
    request[5] = (uint8_t)(count & 0xff);
    crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(crc & 0xff);
    request[7] = (uint8_t)((crc >> 8) & 0xff);

    if (write_frame(t, request, sizeof(request)) != 0)
        return -1;

    if (read_response_prefix(t, slave_id, 0x03, prefix) != 0)
        return -1;

    if (prefix[2] != (uint8_t)(count * 2)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected FC03 byte count %u (expected %d)",
            (unsigned)prefix[2],
            count * 2
        );
        return -1;
    }

    frame[0] = prefix[0];
    frame[1] = prefix[1];
    frame[2] = prefix[2];
    total = 3U + (size_t)prefix[2] + 2U;

    if (read_exact(t, &frame[3], total - 3U) != 0)
        return -1;

    if (t->debug)
        debug_bytes("RX ", frame, total);

    if (validate_crc(t, frame, total) != 0)
        return -1;

    for (i = 0; i < count; ++i) {
        dest[i] = ((uint16_t)frame[3 + 2 * i] << 8) |
                  (uint16_t)frame[4 + 2 * i];
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
    uint8_t request[8];
    uint8_t response[8];
    uint16_t crc;

    if (!t || !valid_slave(slave_id) || address < 0 || address > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC06 arguments");
        return -1;
    }

    request[0] = (uint8_t)slave_id;
    request[1] = 0x06;
    request[2] = (uint8_t)((address >> 8) & 0xff);
    request[3] = (uint8_t)(address & 0xff);
    request[4] = (uint8_t)((value >> 8) & 0xff);
    request[5] = (uint8_t)(value & 0xff);
    crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(crc & 0xff);
    request[7] = (uint8_t)((crc >> 8) & 0xff);

    if (write_frame(t, request, sizeof(request)) != 0)
        return -1;

    if (read_exact(t, response, sizeof(response)) != 0)
        return -1;

    if (t->debug)
        debug_bytes("RX ", response, sizeof(response));

    if (validate_crc(t, response, sizeof(response)) != 0)
        return -1;

    if (response[0] == (uint8_t)slave_id && response[1] == 0x86) {
        snprintf(t->last_error, sizeof(t->last_error),
                 "Modbus exception code %u", (unsigned)response[2]);
        return -1;
    }

    if (memcmp(request, response, 6) != 0) {
        snprintf(t->last_error, sizeof(t->last_error),
                 "FC06 response does not echo request");
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
    uint8_t request[260];
    uint8_t response[8];
    uint16_t crc;
    size_t request_length;
    int i;

    if (!t || !values || !valid_slave(slave_id) ||
        address < 0 || address > 65535 ||
        count < 1 || count > 123 || address + count - 1 > 65535) {
        if (t)
            snprintf(t->last_error, sizeof(t->last_error), "Invalid FC16 arguments");
        return -1;
    }

    request[0] = (uint8_t)slave_id;
    request[1] = 0x10;
    request[2] = (uint8_t)((address >> 8) & 0xff);
    request[3] = (uint8_t)(address & 0xff);
    request[4] = (uint8_t)((count >> 8) & 0xff);
    request[5] = (uint8_t)(count & 0xff);
    request[6] = (uint8_t)(count * 2);

    for (i = 0; i < count; ++i) {
        request[7 + 2 * i] = (uint8_t)((values[i] >> 8) & 0xff);
        request[8 + 2 * i] = (uint8_t)(values[i] & 0xff);
    }

    request_length = 7U + (size_t)count * 2U;
    crc = modbus_crc16(request, request_length);
    request[request_length++] = (uint8_t)(crc & 0xff);
    request[request_length++] = (uint8_t)((crc >> 8) & 0xff);

    if (write_frame(t, request, request_length) != 0)
        return -1;

    if (read_exact(t, response, sizeof(response)) != 0)
        return -1;

    if (t->debug)
        debug_bytes("RX ", response, sizeof(response));

    if (validate_crc(t, response, sizeof(response)) != 0)
        return -1;

    if (response[0] == (uint8_t)slave_id && response[1] == 0x90) {
        snprintf(t->last_error, sizeof(t->last_error),
                 "Modbus exception code %u", (unsigned)response[2]);
        return -1;
    }

    if (response[0] != (uint8_t)slave_id || response[1] != 0x10 ||
        response[2] != request[2] || response[3] != request[3] ||
        response[4] != request[4] || response[5] != request[5]) {
        snprintf(t->last_error, sizeof(t->last_error),
                 "Unexpected FC16 response");
        return -1;
    }

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return 0;
}

const char *modbus_transport_last_error(modbus_transport_t *t)
{
    return t ? t->last_error : "No Modbus transport";
}

#endif /* _WIN32 */
