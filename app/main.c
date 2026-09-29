#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include "cci_inverter.h"
#include "huawei_sun2000.h"
#include "modbus_transport.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
static void sleep_ms(unsigned int ms) { Sleep(ms); }
#define DEFAULT_PORT "COM2"
#else
#include <time.h>
static void sleep_ms(unsigned int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000U;
    ts.tv_nsec = (long)(ms % 1000U) * 1000000L;
    nanosleep(&ts, NULL);
}
#define DEFAULT_PORT "/dev/rs485_2_uart"
#endif

typedef struct {
    const char *port;
    int baud;
    int slave;
    int poll_ms;
    int once;
    int verify_writes;
    int enable_writes;
    int kernel_rs485;
    int debug;

    int have_p_kw;
    double p_kw;
    int have_pf;
    double pf;
    int have_qs;
    double qs;
} app_options_t;

static void usage(const char *exe)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "Connection:\n"
        "  --port <device>       Windows COM2, OpenWrt /dev/rs485_2_uart\n"
        "  --baud <n>            Default 9600\n"
        "  --slave <1..247>      Default 1\n"
        "  --poll-ms <n>         Default 1000\n"
        "  --kernel-rs485        Ask Linux/libmodbus for kernel RS485 mode\n"
        "  --debug               Enable libmodbus frame debug\n"
        "\n"
        "Control (writes are blocked unless --enable-writes is present):\n"
        "  --enable-writes       Explicit safety gate for register writes\n"
        "  --no-verify           Do not FC03 read-back after a write\n"
        "  --set-p-kw <kW>       Huawei 40126-40127, U32 W, FC16\n"
        "  --set-pf <value>      Huawei 40122, I16 x1000, FC06\n"
        "  --set-qs <ratio>      Huawei 40123, I16 Q/S x1000, FC06\n"
        "\n"
        "Run mode:\n"
        "  --once                Read one measurement sample and exit\n"
        "  --help                Show this help\n",
        exe
    );
}

static int parse_int(const char *s, int *out)
{
    char *end = NULL;
    long v;
    errno = 0;
    v = strtol(s, &end, 10);
    if (errno || !end || *end != '\0' || v < -2147483647L || v > 2147483647L)
        return -1;
    *out = (int)v;
    return 0;
}

static int parse_double_value(const char *s, double *out)
{
    char *end = NULL;
    double v;
    errno = 0;
    v = strtod(s, &end);
    if (errno || !end || *end != '\0')
        return -1;
    *out = v;
    return 0;
}

static int parse_args(int argc, char **argv, app_options_t *o)
{
    int i;

    memset(o, 0, sizeof(*o));
    o->port = DEFAULT_PORT;
    o->baud = 9600;
    o->slave = 1;
    o->poll_ms = 1000;
    o->verify_writes = 1;

    for (i = 1; i < argc; ++i) {
        const char *a = argv[i];

        if (strcmp(a, "--help") == 0) {
            usage(argv[0]);
            return 1;
        } else if (strcmp(a, "--port") == 0 && i + 1 < argc) {
            o->port = argv[++i];
        } else if (strcmp(a, "--baud") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], &o->baud) != 0) return -1;
        } else if (strcmp(a, "--slave") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], &o->slave) != 0) return -1;
        } else if (strcmp(a, "--poll-ms") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], &o->poll_ms) != 0) return -1;
        } else if (strcmp(a, "--once") == 0) {
            o->once = 1;
        } else if (strcmp(a, "--enable-writes") == 0) {
            o->enable_writes = 1;
        } else if (strcmp(a, "--no-verify") == 0) {
            o->verify_writes = 0;
        } else if (strcmp(a, "--kernel-rs485") == 0) {
            o->kernel_rs485 = 1;
        } else if (strcmp(a, "--debug") == 0) {
            o->debug = 1;
        } else if (strcmp(a, "--set-p-kw") == 0 && i + 1 < argc) {
            if (parse_double_value(argv[++i], &o->p_kw) != 0) return -1;
            o->have_p_kw = 1;
        } else if (strcmp(a, "--set-pf") == 0 && i + 1 < argc) {
            if (parse_double_value(argv[++i], &o->pf) != 0) return -1;
            o->have_pf = 1;
        } else if (strcmp(a, "--set-qs") == 0 && i + 1 < argc) {
            if (parse_double_value(argv[++i], &o->qs) != 0) return -1;
            o->have_qs = 1;
        } else {
            fprintf(stderr, "Unknown or incomplete option: %s\n", a);
            return -1;
        }
    }

    if (o->slave < 1 || o->slave > 247 || o->baud <= 0 || o->poll_ms < 10)
        return -1;

    if ((o->have_p_kw || o->have_pf || o->have_qs) && !o->enable_writes) {
        fprintf(stderr,
            "Write command requested but --enable-writes is missing.\n"
            "This guard is intentional so a field inverter is not changed accidentally.\n");
        return -1;
    }

    return 0;
}

static void print_info(const cci_inverter_info_t *i)
{
    printf("\n--- Inverter info ---\n");
    printf("Vendor    : %s\n", i->vendor);
    printf("Model     : %s\n", i->model);
    printf("Model ID  : %u\n", (unsigned)i->model_id);
    printf("Strings   : %u\n", (unsigned)i->number_of_strings);
    printf("MPPTs     : %u\n", (unsigned)i->number_of_mppts);
    printf("Pn        : %.3f kW\n", i->rated_power_kw);
    printf("Pmax      : %.3f kW\n", i->max_active_power_kw);
    printf("Smax(inv) : %.3f kVA\n", i->max_apparent_power_kva);
    printf("Qmax +    : %.3f kvar\n", i->qmax_feed_to_grid_kvar);
    printf("Qmax -    : %.3f kvar\n", i->qmax_absorb_from_grid_kvar);
}

static void print_measurements(const cci_inverter_measurements_t *m)
{
    printf(
        "P=%7.2f kW | Q=%7.2f kvar | "
        "V=%6.1f/%6.1f/%6.1f V | "
        "I=%6.1f/%6.1f/%6.1f A | "
        "f=%5.2f Hz | PF=%6.3f | "
        "DC=%7.2f kW | T=%5.1f C | "
        "status=0x%04X | fault=%u\n",
        m->active_power_kw,
        m->reactive_power_kvar,
        m->vab_v, m->vbc_v, m->vca_v,
        m->ia_a, m->ib_a, m->ic_a,
        m->grid_frequency_hz,
        m->power_factor,
        m->dc_power_kw,
        m->internal_temperature_c,
        (unsigned)m->device_status_raw,
        (unsigned)m->fault_code
    );
}

static int apply_requested_writes(
    const app_options_t *o,
    cci_inverter_t *inv,
    modbus_transport_t *transport
)
{
    int rc;

    if (o->have_p_kw) {
        printf("\n[WRITE] active power = %.3f kW\n", o->p_kw);
        rc = cci_inverter_set_active_power_kw(inv, o->p_kw);
        if (rc != CCI_INV_OK) {
            fprintf(stderr, "Active-power write failed (%d): %s\n",
                    rc, modbus_transport_last_error(transport));
            return rc;
        }
        printf("[WRITE] active-power command accepted%s\n",
               o->verify_writes ? " and read-back verified" : "");
    }

    if (o->have_pf) {
        printf("\n[WRITE] power factor = %.3f\n", o->pf);
        rc = cci_inverter_set_power_factor(inv, o->pf);
        if (rc != CCI_INV_OK) {
            fprintf(stderr, "Power-factor write failed (%d): %s\n",
                    rc, modbus_transport_last_error(transport));
            return rc;
        }
        printf("[WRITE] power-factor command accepted%s\n",
               o->verify_writes ? " and read-back verified" : "");
    }

    if (o->have_qs) {
        printf("\n[WRITE] Q/S = %.3f\n", o->qs);
        rc = cci_inverter_set_reactive_qs(inv, o->qs);
        if (rc != CCI_INV_OK) {
            fprintf(stderr, "Q/S write failed (%d): %s\n",
                    rc, modbus_transport_last_error(transport));
            return rc;
        }
        printf("[WRITE] Q/S command accepted%s\n",
               o->verify_writes ? " and read-back verified" : "");
    }

    return CCI_INV_OK;
}

int main(int argc, char **argv)
{
    app_options_t o;
    modbus_rtu_config_t cfg;
    modbus_transport_t *transport;
    huawei_sun2000_t huawei;
    cci_inverter_t inverter;
    cci_inverter_info_t info;
    cci_inverter_measurements_t m;
    int parse_rc;

    parse_rc = parse_args(argc, argv, &o);
    if (parse_rc > 0)
        return 0;
    if (parse_rc < 0) {
        usage(argv[0]);
        return 2;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.serial_port = o.port;
    cfg.baud_rate = o.baud;
    cfg.parity = 'N';
    cfg.data_bits = 8;
    cfg.stop_bits = 1;
    cfg.response_timeout_ms = 1500;
    cfg.byte_timeout_ms = 0;
    cfg.request_kernel_rs485 = o.kernel_rs485;
    cfg.debug = o.debug;

    printf("CCI inverter gateway - Huawei SUN2000\n");
    printf("Driver   : huawei_sun2000\n");
    printf("Port     : %s\n", cfg.serial_port);
    printf("Serial   : %d 8%c%d\n", cfg.baud_rate, cfg.parity, cfg.stop_bits);
    printf("Slave ID : %d\n", o.slave);
    printf("Writes   : %s\n", o.enable_writes ? "ENABLED" : "disabled");

    transport = modbus_transport_open_rtu(&cfg);
    if (!transport) {
        fprintf(stderr,
            "Failed to open libmodbus RTU transport. Check serial port and libmodbus setup.\n");
        return 1;
    }

    if (huawei_sun2000_init(&huawei, transport, o.slave, o.verify_writes) != CCI_INV_OK) {
        fprintf(stderr, "Huawei driver init failed.\n");
        modbus_transport_close(transport);
        return 1;
    }
    huawei_sun2000_bind(&huawei, &inverter);

    if (cci_inverter_read_info(&inverter, &info) != CCI_INV_OK) {
        fprintf(stderr, "Failed to read inverter info: %s\n",
                modbus_transport_last_error(transport));
        modbus_transport_close(transport);
        return 1;
    }

    print_info(&info);

    if (info.model_id != 150) {
        fprintf(stderr,
            "WARNING: this driver profile targets SUN2000-100KTL-M2 model ID 150; got %u.\n",
            (unsigned)info.model_id);
    }

    if (apply_requested_writes(&o, &inverter, transport) != CCI_INV_OK) {
        modbus_transport_close(transport);
        return 1;
    }

    printf("\n--- Live measurements ---\n");

    do {
        if (cci_inverter_read_measurements(&inverter, &m) == CCI_INV_OK) {
            print_measurements(&m);
        } else {
            fprintf(stderr, "Measurement read failed: %s\n",
                    modbus_transport_last_error(transport));
        }

        if (!o.once)
            sleep_ms((unsigned int)o.poll_ms);
    } while (!o.once);

    modbus_transport_close(transport);
    return 0;
}
