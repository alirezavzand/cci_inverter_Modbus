cmake_minimum_required(VERSION 3.16)
project(cci_inverter_gateway VERSION 0.1.1 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(CCI_BUILD_TESTS "Build driver unit tests" ON)

add_library(cci_inverter_core STATIC
    src/cci_inverter.c
)
target_include_directories(cci_inverter_core PUBLIC include)

# Transport backend:
# - Windows: native Win32 serial RTU backend. This avoids a libmodbus
#   3.1.12/3.2.0 Windows-RTU receive regression around ctx->s == -1.
# - Linux/OpenWrt: libmodbus, which is the production embedded backend.
if(WIN32)
    add_library(cci_modbus_transport STATIC
        src/modbus_transport_win32.c
    )
    target_include_directories(cci_modbus_transport PUBLIC include)
else()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBMODBUS QUIET IMPORTED_TARGET libmodbus)
    endif()

    if(TARGET PkgConfig::LIBMODBUS)
        set(CCI_LIBMODBUS_TARGET PkgConfig::LIBMODBUS)
    else()
        find_path(LIBMODBUS_INCLUDE_DIR
            NAMES modbus.h
            PATH_SUFFIXES modbus
            HINTS ${LIBMODBUS_ROOT} ENV LIBMODBUS_ROOT
        )
        find_library(LIBMODBUS_LIBRARY
            NAMES modbus libmodbus
            HINTS ${LIBMODBUS_ROOT} ENV LIBMODBUS_ROOT
            PATH_SUFFIXES lib lib64
        )

        if(NOT LIBMODBUS_INCLUDE_DIR OR NOT LIBMODBUS_LIBRARY)
            message(FATAL_ERROR
                "libmodbus not found. Install libmodbus or set LIBMODBUS_ROOT/CMAKE_PREFIX_PATH."
            )
        endif()

        add_library(cci_libmodbus UNKNOWN IMPORTED)
        set_target_properties(cci_libmodbus PROPERTIES
            IMPORTED_LOCATION "${LIBMODBUS_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBMODBUS_INCLUDE_DIR}"
        )
        set(CCI_LIBMODBUS_TARGET cci_libmodbus)
    endif()

    add_library(cci_modbus_transport STATIC
        src/modbus_transport_libmodbus.c
    )
    target_include_directories(cci_modbus_transport PUBLIC include)
    target_link_libraries(cci_modbus_transport PUBLIC ${CCI_LIBMODBUS_TARGET})
endif()

add_library(cci_driver_huawei STATIC
    drivers/huawei/huawei_sun2000.c
)
target_include_directories(cci_driver_huawei
    PUBLIC include drivers/huawei
)
target_link_libraries(cci_driver_huawei PUBLIC cci_inverter_core cci_modbus_transport)
if(NOT WIN32)
    target_link_libraries(cci_driver_huawei PUBLIC m)
endif()

add_executable(cci_inverter_gateway app/main.c)
target_include_directories(cci_inverter_gateway PRIVATE include drivers/huawei)
target_link_libraries(cci_inverter_gateway PRIVATE cci_driver_huawei)

if(MSVC)
    target_compile_options(cci_inverter_core PRIVATE /W4)
    target_compile_options(cci_modbus_transport PRIVATE /W4)
    target_compile_options(cci_driver_huawei PRIVATE /W4)
    target_compile_options(cci_inverter_gateway PRIVATE /W4)
else()
    foreach(tgt cci_inverter_core cci_modbus_transport cci_driver_huawei cci_inverter_gateway)
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

if(CCI_BUILD_TESTS)
    enable_testing()
    add_executable(test_huawei_driver
        tests/test_huawei_driver.c
        tests/fake_modbus_transport.c
        src/cci_inverter.c
        drivers/huawei/huawei_sun2000.c
    )
    target_include_directories(test_huawei_driver PRIVATE include drivers/huawei tests)
    if(NOT WIN32)
        target_link_libraries(test_huawei_driver PRIVATE m)
    endif()
    add_test(NAME huawei_driver COMMAND test_huawei_driver)
endif()
