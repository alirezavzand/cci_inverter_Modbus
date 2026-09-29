# CCI Inverter Gateway v1

A modular C foundation for the CCI plant controller. It keeps **Modbus**,
**vendor-specific inverter mapping**, and future **CCI/CEI control logic** in
separate layers.

```text
future IEC 61850 / CEI control engine
                |
        cci_inverter API
                |
       vendor driver layer
        /              \
 Huawei SUN2000       future SMA/etc.
        |
   Modbus transport
        |
     libmodbus
        |
 Windows COM / OpenWrt serial / RS485
```

## Windows transport note (v0.1.1)

On Windows this project intentionally uses a small native Win32 Modbus RTU transport implementation behind the same `modbus_transport.h` API. Current libmodbus 3.1.12/3.2.0 Windows RTU builds can send a request but fail before receiving with `ERROR Invalid socket descriptor -1`; the Windows RTU backend uses a Win32 serial `HANDLE` while the common receive path rejects `ctx->s == -1`.

This does **not** change the inverter driver or application API. The Huawei driver and all higher layers are identical on Windows and OpenWrt. On Linux/OpenWrt the transport backend remains libmodbus.

Windows build therefore needs only Visual Studio + CMake:

```cmd
cmake -S . -B build -A x64
cmake --build build --config Release
```

OpenWrt/Linux still requires libmodbus and uses `src/modbus_transport_libmodbus.c`.

## What this version does

- Reads the same Huawei SUN2000-100KTL-M2 values as the previous reader.
- Uses **libmodbus** on both Windows and OpenWrt/Linux.
- Adds FC06 and FC16 write support.
- Uses the **real Huawei writable register addresses**, not mock-only addresses.
- Verifies writes by reading the exact register(s) back by default.
- Keeps the slave ID per inverter transaction, so one RS485 bus can later host
  several inverter objects with different slave IDs.
- Adds a generic `cci_inverter_t` interface so another manufacturer can be added
  as another driver without changing the future CEI control engine.
- Includes a test-only fake transport so driver encoding/decoding can be tested
  without serial hardware.

## Huawei registers used now

Monitoring/nameplate:

- 30000 model string
- 30070 model ID
- 30071 number of strings
- 30072 number of MPPTs
- 30073 rated power
- 30075 max active power
- 30077 max apparent power
- 30079/30081 reactive capability
- 32064..32090 real-time measurement block

Writable controls implemented in C:

- `40126-40127` — active power fixed-value derating, U32 watts, FC16
- `40122` — power factor, I16 gain 1000, FC06
- `40123` — reactive power compensation Q/S, I16 gain 1000, FC06

The supplied Huawei map also contains mode/scheduling registers such as 42014
and 42046, but this version **does not invent enum values that are not defined in
our source material**. Those will be added only after their exact firmware value
mapping is confirmed.

## Safety gate

The program starts read-only. A write is rejected unless the command line also
contains `--enable-writes`.

That is intentional. The same executable can later point at a real inverter, so
an accidental test command should not alter plant power.

## Windows build (Visual Studio + vcpkg)

The repository includes `vcpkg.json` with a `libmodbus` dependency.

```powershell
cmake -S . -B build `
  -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

If libmodbus is installed manually instead, point CMake at it with either
`LIBMODBUS_ROOT` or `CMAKE_PREFIX_PATH`.

Run against the Python mock on COM1 through your virtual COM pair:

```powershell
python tools/sun2000_100ktl_m2_rtu_real_register_mock.py --port COM1

.\build\Release\cci_inverter_gateway.exe `
  --port COM2 --baud 9600 --slave 1 --once
```

Active-power write test:

```powershell
.\build\Release\cci_inverter_gateway.exe `
  --port COM2 --baud 9600 --slave 1 `
  --enable-writes --set-p-kw 75 --once
```

The C driver writes `75000 W` to Huawei registers `40126-40127` using FC16,
then reads the same two registers back and verifies the raw words.

Other current write tests:

```powershell
# PF register 40122, raw 950
.\build\Release\cci_inverter_gateway.exe --port COM2 --slave 1 `
  --enable-writes --set-pf 0.95 --once

# Q/S register 40123, raw -200
.\build\Release\cci_inverter_gateway.exe --port COM2 --slave 1 `
  --enable-writes --set-qs -0.20 --once
```

`--set-qs` is intentionally an inverter-level Q/S command. It is **not** the CEI
plant Smax calculation. The future CCI control layer will convert plant-level
requests into inverter-level commands.

## OpenWrt

The exact same `huawei_sun2000.c` and libmodbus transport are used.

A simple toolchain build is provided:

```sh
make -f Makefile.openwrt \
  CC=<openwrt-toolchain-gcc> \
  LIBMODBUS_CFLAGS='-I<staging>/usr/include/modbus' \
  LIBMODBUS_LIBS='-L<staging>/usr/lib -lmodbus'
```

Typical run on the TesPro target:

```sh
./cci_inverter_gateway \
  --port /dev/rs485_2_uart --baud 9600 --slave 1
```

If the Linux serial driver needs the kernel RS485 ioctl mode:

```sh
./cci_inverter_gateway \
  --port /dev/rs485_2_uart --baud 9600 --slave 1 --kernel-rs485
```

Do not enable that option automatically: some RS485 adapters/drivers already
handle direction internally.

## Adding another inverter later

Create another driver, for example:

```text
drivers/sma/sma_inverter.c
drivers/sma/sma_inverter.h
```

and implement the same `cci_inverter_ops_t` functions. The CCI layer then calls:

```c
cci_inverter_read_measurements(&inv, &m);
cci_inverter_set_active_power_kw(&inv, 75.0);
```

without knowing whether the physical device is Huawei, SMA, or another vendor.

## Current boundary

This project is the **device/transport layer**. It does not yet implement CEI
mode arbitration, Q(V), cosphi(P), priority logic, plant Smax, or IEC 61850
control objects. Those should be built above `cci_inverter_t`, not inside the
Huawei driver.


## Windows virtual COM note

Some virtual null-modem drivers return Win32 `ERROR_INVALID_FUNCTION` (1) for `FlushFileBuffers()` even after a successful serial `WriteFile()`. The Windows transport treats only `ERROR_INVALID_FUNCTION` and `ERROR_NOT_SUPPORTED` from that drain call as non-fatal so COM-pair simulator testing can continue. Other serial errors remain fatal. OpenWrt uses the libmodbus backend and is unaffected.
