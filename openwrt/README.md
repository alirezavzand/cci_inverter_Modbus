# OpenWrt build notes

The production transport is `src/modbus_transport_libmodbus.c`; there is no
OpenWrt-specific Huawei driver. This is intentional: Windows and OpenWrt use
exactly the same driver and the same Huawei register map.

## Direct cross-compile

Use the OpenWrt SDK/toolchain compiler and the libmodbus headers/library from the
same target staging directory:

```sh
make -f Makefile.openwrt \
  CC="$TARGET_CC" \
  LIBMODBUS_CFLAGS="-I$STAGING_DIR/usr/include/modbus" \
  LIBMODBUS_LIBS="-L$STAGING_DIR/usr/lib -lmodbus"
```

On the TG544 the runtime port may be `/dev/rs485_2_uart` (verify the final
TesPro production mapping). If the kernel serial driver itself requires RS485
mode, add `--kernel-rs485`; otherwise leave it off.

A proper OpenWrt package recipe can be added once the exact SDK release/target
for the TesPro image is frozen. The source layout is already suitable for it.
