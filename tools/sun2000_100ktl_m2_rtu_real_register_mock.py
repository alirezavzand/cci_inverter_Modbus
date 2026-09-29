#!/usr/bin/env python3
"""
Huawei SUN2000-100KTL-M2 Modbus RTU simulator
---------------------------------------------

- Serial protocol: Modbus RTU
- Default Windows serial port: COM1
- Default slave/device address: 1
- Default serial settings: 9600 baud, 8 data bits, no parity, 1 stop bit
- Uses modern PyModbus SimData / SimDevice API (no deprecated datastore API)
- Simulates ONE SUN2000-100KTL-M2 connected to PV strings
- Monitoring registers plus writable Huawei control-register mock
- Control writes are acknowledged/logged only; they do not yet change P/Q

IMPORTANT:
The serial settings are simulator defaults for testing. The real inverter and CCI
must later be configured with matching RS485 settings.

Huawei register addresses used here come from:
"Solar Inverter Modbus Interface Definitions (V3.0)", Issue 01, 2023-01-17.
"""

from __future__ import annotations

import argparse
import math
import random
import time

from pymodbus import FramerType, ModbusDeviceIdentification
from pymodbus.server import StartSerialServer
from pymodbus.simulator import DataType, SimData, SimDevice


# ---------------------------------------------------------------------------
# Huawei register addresses
# ---------------------------------------------------------------------------

MODEL = 30000                 # STR, 15 registers
MODEL_ID = 30070              # U16
NUMBER_OF_STRINGS = 30071     # U16
NUMBER_OF_MPPTS = 30072       # U16
RATED_POWER = 30073           # U32, kW, gain 1000
MAX_ACTIVE_POWER = 30075      # U32, kW, gain 1000
MAX_APPARENT_POWER = 30077    # U32, kVA, gain 1000
QMAX_FEED_TO_GRID = 30079     # I32, kvar, gain 1000
QMAX_ABSORB_FROM_GRID = 30081 # I32, kvar, gain 1000

PV1_VOLTAGE = 32016           # I16, V, gain 10
PV1_CURRENT = 32017           # I16, A, gain 100
DC_POWER = 32064              # I32, kW, gain 1000

VAB = 32066                   # U16, V, gain 10
VBC = 32067                   # U16, V, gain 10
VCA = 32068                   # U16, V, gain 10

IA = 32072                    # I32, A, gain 1000
IB = 32074                    # I32, A, gain 1000
IC = 32076                    # I32, A, gain 1000

ACTIVE_POWER = 32080          # I32, kW, gain 1000
REACTIVE_POWER = 32082        # I32, kvar, gain 1000
POWER_FACTOR = 32084          # I16, gain 1000
GRID_FREQUENCY = 32085        # U16, Hz, gain 100
INVERTER_EFFICIENCY = 32086   # U16, %, gain 100
INTERNAL_TEMPERATURE = 32087  # I16, degC, gain 10
DEVICE_STATUS = 32089         # raw E16
FAULT_CODE = 32090            # U16

CUMULATIVE_ENERGY = 32106     # U32, kWh, gain 100
DAILY_ENERGY = 32114          # U32, kWh, gain 100


# ---------------------------------------------------------------------------
# Huawei writable control registers used by the future CCI control layer
# ---------------------------------------------------------------------------
#
# These are real Huawei register addresses from the supplied Modbus map.
# In THIS simulator version, writes are accepted and stored, but they do not
# yet change the electrical output. The simulator only reports that the
# corresponding control/mode has been received/activated.

Q_U_CURVE_MODEL = 40037                 # E16, Q-U characteristic curve model
Q_U_TRIGGER_POWER_PERCENT = 40038       # I16, %
ACTIVE_POWER_FIXED_DERATING_LP = 40120  # U16, kW, gain 10
POWER_FACTOR_SETPOINT = 40122           # I16, gain 1000
REACTIVE_POWER_QS_SETPOINT = 40123      # I16, Q/S, gain 1000
REACTIVE_POWER_ADJ_TIME = 40124         # U16, s
ACTIVE_POWER_PERCENT_DERATING = 40125   # I16, %, gain 10
ACTIVE_POWER_FIXED_DERATING = 40126     # U32, W

COSPHI_P_CURVE_START = 40133            # 21-register MLD block
COSPHI_P_CURVE_END = 40153
Q_U_CURVE_START = 40154                 # 21-register MLD block
Q_U_CURVE_END = 40174
PF_U_CURVE_START = 40175                # 21-register MLD block
PF_U_CURVE_END = 40195
Q_P_CURVE_START = 40354                 # 21-register MLD block
Q_P_CURVE_END = 40374

REMOTE_POWER_SCHEDULING = 42014         # E16
REACTIVE_POWER_GRADIENT = 42015         # U32, %/s, gain 1000
ACTIVE_POWER_GRADIENT = 42017           # U32, %/s, gain 1000
SCHEDULING_MAINTENANCE_TIME = 42019     # U32, s
MAX_APPARENT_POWER_RW = 42021            # U32, kVA, gain 1000
MAX_ACTIVE_POWER_RW = 42023              # U32, kW, gain 1000
APPARENT_POWER_REFERENCE = 42025         # U32, gain 1000
ACTIVE_POWER_REFERENCE = 42027           # U32, kW, gain 1000
PQ_MODE = 42046                          # E16

# Only documented Huawei RW registers used by this CCI test are writable.
# Everything else stays unmapped/non-writable, so the mock behaves much more
# like a real inverter instead of accepting arbitrary addresses.
#
# Each tuple is: (start_address, register_count, description)
HUAWEI_WRITABLE_RANGES = [
    (40000, 2,  "System Time [Local Time]"),
    (40037, 2,  "Q-U mode + Q-U trigger power"),
    (40120, 1,  "Active power fixed-value derating [low precision]"),
    (40122, 6,  "PF / Q-S / adjustment time / active-power derating"),
    (40128, 3,  "Night reactive-power controls"),
    (40133, 66, "cosphi-P, Q-U, PF-U curves and related parameters"),
    (40354, 23, "Q-P curve + Q-U limiting/delay parameters"),
    (42000, 4,  "Grid code / output mode / nominal V / nominal f"),
    (42014, 15, "Remote power scheduling and P/Q scheduling references"),
    (42046, 1,  "PQ mode"),
]


# ---------------------------------------------------------------------------
# SUN2000-100KTL-M2 data
# ---------------------------------------------------------------------------

MODEL_NAME = "SUN2000-100KTL-M2"
MODEL_ID_VALUE = 150

PN_KW = 100.0
PMAX_KW = 110.0
SMAX_KVA = 110.0

N_MPPT = 10
N_STRINGS = 20

# Test-only Q capability derived from S=110 kVA and |PF|min=0.8:
# Q = S * sin(acos(0.8)) = 66 kvar.
# This is NOT intended to replace a full manufacturer P-Q capability curve.
MOCK_QMAX_KVAR = 66.0

REGISTER_COUNT = 40000


# ---------------------------------------------------------------------------
# Raw Modbus encoding helpers
# Huawei U32/I32 values occupy two registers, high word first.
# ---------------------------------------------------------------------------

def u16(value: int) -> list[int]:
    return [int(value) & 0xFFFF]


def i16(value: int) -> list[int]:
    return [int(value) & 0xFFFF]


def u32(value: int) -> list[int]:
    value = int(value) & 0xFFFFFFFF
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


def i32(value: int) -> list[int]:
    return u32(value)


def str_regs(text: str, register_count: int) -> list[int]:
    raw = text.encode("ascii", errors="replace")[: register_count * 2]
    raw = raw.ljust(register_count * 2, b"\x00")
    return [
        (raw[i] << 8) | raw[i + 1]
        for i in range(0, len(raw), 2)
    ]


def put(registers: list[int], address: int, values: list[int]) -> None:
    end = address + len(values)
    if address < 0 or end > len(registers):
        raise ValueError(f"Register range outside datastore: {address}..{end - 1}")
    registers[address:end] = [v & 0xFFFF for v in values]


# ---------------------------------------------------------------------------
# Dynamic PV / inverter model
# ---------------------------------------------------------------------------

class Sun2000State:
    def __init__(
        self,
        voltage_v: float,
        q_kvar: float,
        mode: str,
        fixed_power_kw: float,
        seed: int,
        raw_status: int,
    ):
        self.voltage_v = voltage_v
        self.q_kvar = q_kvar
        self.mode = mode
        self.fixed_power_kw = fixed_power_kw
        self.raw_status = raw_status

        self.rng = random.Random(seed)
        self.started = time.monotonic()
        self.last_update = self.started

        self.energy_kwh = 0.0
        self.daily_energy_kwh = 0.0
        self.last_print = 0.0

    def solar_fraction(self, elapsed: float) -> float:
        if self.mode == "fixed":
            return max(0.0, min(1.0, self.fixed_power_kw / PN_KW))

        if self.mode == "daycycle":
            # One simulated sunrise-to-sunset cycle every 120 seconds.
            phase = (elapsed % 120.0) / 120.0
            sun = max(0.0, math.sin(math.pi * phase))
            cloud = (
                0.92
                + 0.06 * math.sin(elapsed / 6.5)
                + self.rng.uniform(-0.015, 0.015)
            )
            return max(0.0, min(1.0, sun * cloud))

        # "daytime": continuously generating with changing irradiance/clouds.
        value = (
            0.78
            + 0.12 * math.sin(elapsed / 18.0)
            + 0.04 * math.sin(elapsed / 4.5)
            + self.rng.uniform(-0.008, 0.008)
        )
        return max(0.15, min(1.0, value))

    def update_registers(self, registers: list[int], start_address: int) -> None:
        now = time.monotonic()
        elapsed = now - self.started
        dt_h = max(0.0, now - self.last_update) / 3600.0
        self.last_update = now

        solar = self.solar_fraction(elapsed)

        p_kw = PN_KW * solar
        if self.mode == "fixed":
            p_kw = max(0.0, min(PMAX_KW, self.fixed_power_kw))

        q_kvar = max(-MOCK_QMAX_KVAR, min(MOCK_QMAX_KVAR, self.q_kvar))

        # Respect this simulated inverter's 110 kVA apparent-power envelope.
        p_from_s = math.sqrt(max(0.0, SMAX_KVA**2 - q_kvar**2))
        p_kw = min(p_kw, p_from_s)

        # Approximate conversion efficiency for test data.
        efficiency = 0.985 - 0.006 * (1.0 - solar)
        efficiency = max(0.95, min(0.988, efficiency))
        dc_power_kw = p_kw / efficiency if p_kw > 0.0 else 0.0

        vab = self.voltage_v + self.rng.uniform(-0.7, 0.7)
        vbc = self.voltage_v + self.rng.uniform(-0.7, 0.7)
        vca = self.voltage_v + self.rng.uniform(-0.7, 0.7)
        vavg = (vab + vbc + vca) / 3.0

        s_kva = math.sqrt(p_kw**2 + q_kvar**2)
        line_current_a = (
            s_kva * 1000.0 / (math.sqrt(3.0) * vavg)
            if vavg > 1.0
            else 0.0
        )

        ia = line_current_a * (1.0 + self.rng.uniform(-0.004, 0.004))
        ib = line_current_a * (1.0 + self.rng.uniform(-0.004, 0.004))
        ic = line_current_a * (1.0 + self.rng.uniform(-0.004, 0.004))

        pf = p_kw / s_kva if s_kva > 0.001 else 1.0
        freq_hz = (
            50.0
            + 0.012 * math.sin(elapsed / 7.0)
            + self.rng.uniform(-0.003, 0.003)
        )
        temp_c = 32.0 + 18.0 * solar + 1.5 * math.sin(elapsed / 25.0)

        self.energy_kwh += p_kw * dt_h
        self.daily_energy_kwh += p_kw * dt_h

        def set_abs(address: int, values: list[int]) -> None:
            offset = address - start_address
            end = offset + len(values)
            if 0 <= offset and end <= len(registers):
                registers[offset:end] = [v & 0xFFFF for v in values]

        # PV strings: SUN2000-100KTL-M2 has 10 MPPTs and up to 20 inputs.
        per_string_kw = dc_power_kw / N_STRINGS
        for i in range(N_STRINGS):
            pv_v = (
                600.0
                + 8.0 * math.sin(elapsed / 15.0 + i * 0.35)
                + self.rng.uniform(-1.0, 1.0)
            )
            pv_i = (
                per_string_kw * 1000.0 / pv_v
                if pv_v > 1.0 and per_string_kw > 0
                else 0.0
            )
            pv_i *= 1.0 + self.rng.uniform(-0.025, 0.025)

            set_abs(PV1_VOLTAGE + 2 * i, i16(round(pv_v * 10)))
            set_abs(PV1_CURRENT + 2 * i, i16(round(pv_i * 100)))

        set_abs(DC_POWER, i32(round(dc_power_kw * 1000)))

        set_abs(VAB, u16(round(vab * 10)))
        set_abs(VBC, u16(round(vbc * 10)))
        set_abs(VCA, u16(round(vca * 10)))

        set_abs(IA, i32(round(ia * 1000)))
        set_abs(IB, i32(round(ib * 1000)))
        set_abs(IC, i32(round(ic * 1000)))

        set_abs(ACTIVE_POWER, i32(round(p_kw * 1000)))
        set_abs(REACTIVE_POWER, i32(round(q_kvar * 1000)))
        set_abs(POWER_FACTOR, i16(round(pf * 1000)))
        set_abs(GRID_FREQUENCY, u16(round(freq_hz * 100)))
        set_abs(INVERTER_EFFICIENCY, u16(round(efficiency * 10000)))
        set_abs(INTERNAL_TEMPERATURE, i16(round(temp_c * 10)))

        set_abs(DEVICE_STATUS, u16(self.raw_status))
        set_abs(FAULT_CODE, u16(0))

        set_abs(CUMULATIVE_ENERGY, u32(round(self.energy_kwh * 100)))
        set_abs(DAILY_ENERGY, u32(round(self.daily_energy_kwh * 100)))

        # Keep terminal output readable: print roughly once per second.
        if now - self.last_print >= 1.0:
            self.last_print = now
            print(
                f"P={p_kw:6.2f} kW | "
                f"Q={q_kvar:6.2f} kvar | "
                f"V={vavg:6.1f} V | "
                f"I={line_current_a:6.1f} A | "
                f"f={freq_hz:5.2f} Hz | "
                f"DC={dc_power_kw:6.2f} kW"
            )



# ---------------------------------------------------------------------------
# Control-write logging
# ---------------------------------------------------------------------------

def control_name_for_address(address: int) -> str | None:
    """Return a friendly name for a Huawei writable control register."""
    if address == Q_U_CURVE_MODEL:
        return "Q(V) / Q-U characteristic mode"
    if address == Q_U_TRIGGER_POWER_PERCENT:
        return "Q(V) / Q-U trigger power"
    if address == ACTIVE_POWER_FIXED_DERATING_LP:
        return "Active-power fixed derating (low precision)"
    if address == POWER_FACTOR_SETPOINT:
        return "Power-factor setpoint"
    if address == REACTIVE_POWER_QS_SETPOINT:
        return "Reactive-power Q/S setpoint"
    if address == REACTIVE_POWER_ADJ_TIME:
        return "Reactive-power adjustment time"
    if address == ACTIVE_POWER_PERCENT_DERATING:
        return "Active-power percentage derating"
    if ACTIVE_POWER_FIXED_DERATING <= address <= ACTIVE_POWER_FIXED_DERATING + 1:
        return "Active-power fixed derating"
    if COSPHI_P_CURVE_START <= address <= COSPHI_P_CURVE_END:
        return "cosphi(P) characteristic curve"
    if Q_U_CURVE_START <= address <= Q_U_CURVE_END:
        return "Q(V) / Q-U characteristic curve"
    if PF_U_CURVE_START <= address <= PF_U_CURVE_END:
        return "PF(U) characteristic curve"
    if Q_P_CURVE_START <= address <= Q_P_CURVE_END:
        return "Q(P) characteristic curve"
    if address == REMOTE_POWER_SCHEDULING:
        return "Remote power scheduling"
    if REACTIVE_POWER_GRADIENT <= address <= REACTIVE_POWER_GRADIENT + 1:
        return "Reactive-power variation gradient"
    if ACTIVE_POWER_GRADIENT <= address <= ACTIVE_POWER_GRADIENT + 1:
        return "Active-power variation gradient"
    if SCHEDULING_MAINTENANCE_TIME <= address <= SCHEDULING_MAINTENANCE_TIME + 1:
        return "Scheduling instruction maintenance time"
    if MAX_APPARENT_POWER_RW <= address <= MAX_APPARENT_POWER_RW + 1:
        return "Maximum apparent power setting"
    if MAX_ACTIVE_POWER_RW <= address <= MAX_ACTIVE_POWER_RW + 1:
        return "Maximum active power setting"
    if APPARENT_POWER_REFERENCE <= address <= APPARENT_POWER_REFERENCE + 1:
        return "Apparent-power scheduling reference"
    if ACTIVE_POWER_REFERENCE <= address <= ACTIVE_POWER_REFERENCE + 1:
        return "Active-power scheduling reference"
    if address == PQ_MODE:
        return "PQ mode"
    return None


def log_control_write(address: int, values: list[int]) -> None:
    """Acknowledge/log a control write without changing electrical behavior."""
    names: list[str] = []
    for i in range(len(values)):
        name = control_name_for_address(address + i)
        if name and name not in names:
            names.append(name)

    if names:
        for name in names:
            print(
                f"[CONTROL] {name}: MODE ACTIVATED "
                f"(write @ {address}, raw={values})"
            )
    else:
        print(
            f"[CONTROL] Huawei writable-register write received "
            f"@ {address}, raw={values} "
            f"(stored; no electrical effect yet)"
        )


# ---------------------------------------------------------------------------
# Datamodel
# ---------------------------------------------------------------------------

def build_initial_registers(raw_status: int) -> list[int]:
    registers = [0] * REGISTER_COUNT

    put(registers, MODEL, str_regs(MODEL_NAME, 15))
    put(registers, MODEL_ID, u16(MODEL_ID_VALUE))
    put(registers, NUMBER_OF_STRINGS, u16(N_STRINGS))
    put(registers, NUMBER_OF_MPPTS, u16(N_MPPT))

    put(registers, RATED_POWER, u32(round(PN_KW * 1000)))
    put(registers, MAX_ACTIVE_POWER, u32(round(PMAX_KW * 1000)))
    put(registers, MAX_APPARENT_POWER, u32(round(SMAX_KVA * 1000)))

    put(registers, QMAX_FEED_TO_GRID, i32(round(MOCK_QMAX_KVAR * 1000)))
    put(registers, QMAX_ABSORB_FROM_GRID, i32(round(-MOCK_QMAX_KVAR * 1000)))

    put(registers, DEVICE_STATUS, u16(raw_status))
    put(registers, FAULT_CODE, u16(0))

    return registers


def build_device(state: Sun2000State, raw_status: int) -> SimDevice:
    initial = build_initial_registers(raw_status)

    # Monitoring/nameplate area: read-only.
    monitoring_block = SimData(
        address=0,
        values=initial,
        datatype=DataType.REGISTERS,
        readonly=True,
    )

    # Writable blocks exist ONLY at real Huawei RW addresses used by this
    # project. The same addresses and FC06/FC16 transactions can later be
    # sent by the CCI to a real SUN2000.
    control_blocks: list[SimData] = []

    for start, count, _description in HUAWEI_WRITABLE_RANGES:
        values = [0] * count

        # Test-only read-back default. 40122 is the real Huawei PF register;
        # 1000 decodes as cos(phi)=1.000. The address/scaling are real, while
        # this initial value is only a simulator default.
        if start <= POWER_FACTOR_SETPOINT < start + count:
            values[POWER_FACTOR_SETPOINT - start] = 1000

        control_blocks.append(
            SimData(
                address=start,
                values=values,
                datatype=DataType.REGISTERS,
                readonly=False,
            )
        )

    async def on_register_access(
        function_code: int,
        start_address: int,
        address: int,
        count: int,
        current_registers: list[int],
        new_registers: list[int] | None,
    ):
        # Reads: only refresh dynamic values in the monitoring block.
        if new_registers is None:
            if start_address == 0:
                state.update_registers(current_registers, start_address)
            return None

        # Writes (FC06/FC16): PyModbus stores the values after callback success.
        # For now we only log/acknowledge the command.
        log_control_write(address, [int(v) & 0xFFFF for v in new_registers])
        return None

    identity = ModbusDeviceIdentification()
    identity.VendorName = "Huawei (simulated)"
    identity.ProductCode = "SUN2000"
    identity.ProductName = "SUN2000-100KTL-M2 Simulator"
    identity.ModelName = MODEL_NAME
    identity.MajorMinorRevision = "CCI-RTU-MOCK-CONTROL-1"

    return SimDevice(
        id=1,
        simdata=[monitoring_block, *control_blocks],
        identity=identity,
        action=on_register_access,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Huawei SUN2000-100KTL-M2 Modbus RTU simulator"
    )

    parser.add_argument("--port", default="COM1", help="Windows serial port, default COM1")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--parity", choices=["N", "E", "O"], default="N")
    parser.add_argument("--stopbits", type=int, choices=[1, 2], default=1)

    parser.add_argument(
        "--mode",
        choices=["daytime", "daycycle", "fixed"],
        default="daytime",
    )
    parser.add_argument("--power-kw", type=float, default=75.0)
    parser.add_argument("--q-kvar", type=float, default=0.0)
    parser.add_argument("--voltage", type=float, default=400.0)
    parser.add_argument("--seed", type=int, default=7)

    parser.add_argument(
        "--device-status",
        type=lambda x: int(x, 0),
        default=0x0200,
        help=(
            "Raw Huawei register 32089 mock status. "
            "Example: --device-status 0x0200"
        ),
    )

    args = parser.parse_args()

    state = Sun2000State(
        voltage_v=args.voltage,
        q_kvar=args.q_kvar,
        mode=args.mode,
        fixed_power_kw=args.power_kw,
        seed=args.seed,
        raw_status=args.device_status,
    )

    device = build_device(state, args.device_status)

    print("Huawei SUN2000-100KTL-M2 Modbus RTU simulator")
    print(f"Port      : {args.port}")
    print(f"Slave ID  : 1")
    print(
        f"Serial    : {args.baudrate} baud, 8{args.parity}{args.stopbits}"
    )
    print("Protocol  : Modbus RTU")
    print("Registers : Huawei V3.0 monitoring + writable control registers")
    print()
    print("Waiting for CCI Modbus requests...")
    print("Writes are accepted only at whitelisted real Huawei RW register addresses.")
    print("Recognized control writes are stored/logged; they do not change P/Q yet.")
    print("Press Ctrl+C to stop.")
    print()

    StartSerialServer(
        context=device,
        framer=FramerType.RTU,
        port=args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity=args.parity,
        stopbits=args.stopbits,
    )


if __name__ == "__main__":
    main()
