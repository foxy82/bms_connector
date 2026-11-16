"""Modbus RTU response parser for Seplos BMS V3 (PIA / PIB tables).

PIA  — registers 0x1000–0x1011 (18 regs) — pack-level data
PIB  — registers 0x1100–0x1119 (26 regs) — cell voltages + temperatures
"""

import struct

try:
    from .vendor_spec import PIC_BITS, SFA_BITS, SPA_FIELDS
except ImportError:  # standalone test loading
    from vendor_spec import PIC_BITS, SFA_BITS, SPA_FIELDS
import logging

_LOGGER = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# CRC-16 Modbus
# ---------------------------------------------------------------------------

def modbus_crc(data: bytes) -> bytes:
    """Calculate the Modbus RTU CRC (little-endian, 2 bytes)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


def verify_crc(frame_hex: str) -> bool:
    """Validate CRC for an entire Modbus frame given as hex string."""
    try:
        raw = bytes.fromhex(frame_hex)
        if len(raw) < 4:
            return False
        payload = raw[:-2]
        received_crc = raw[-2:]
        expected_crc = modbus_crc(payload)
        return received_crc == expected_crc
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Command construction
# ---------------------------------------------------------------------------

def build_read_command(addr: int, register: int, count: int, function: int = 0x04) -> str:
    """Build a Modbus RTU 0x04 (Read Input Registers) command with CRC.

    Args:
        addr:     Slave address (0x01–0x7F).
        register: Start register address (e.g. 0x1000 for PIA).
        count:    Number of registers (or coils) to read.
        function: Modbus function code: 0x04 registers, 0x01 coils.

    Returns:
        Complete command as a hex string (without spaces), CRC included.

    """
    payload = bytes([addr, function]) + struct.pack('>HH', register, count)
    crc = modbus_crc(payload)
    cmd = (payload + crc).hex()
    _LOGGER.debug("Modbus command built: %s (addr=0x%02X, reg=0x%04X, count=%d)",
                  cmd, addr, register, count)
    return cmd


def build_commands_for_address(battery_addr: int) -> list:
    """Build the V3 read commands for PIA, PIB, VIA, model, PIC, SFA and SPA."""
    commands = [
        build_read_command(battery_addr, 0x1000, 0x12),        # PIA
        build_read_command(battery_addr, 0x1100, 0x1A),        # PIB
        build_read_command(battery_addr, 0x1700, 0x33),        # VIA
        build_read_command(battery_addr, 0x170A, 0x0A),        # part model
        build_read_command(battery_addr, 0x1200, 0x90, 0x01),  # PIC alarms
        build_read_command(battery_addr, 0x1400, 0x50, 0x01),  # SFA switches
        build_read_command(battery_addr, 0x1300, 0x6A),        # SPA settings
    ]
    _LOGGER.debug("Commands for battery 0x%02X: %s", battery_addr, commands)
    return commands


def build_poll_commands(battery_addr: int, identity=True, status=True,
                        settings=True) -> list:
    """(block, command) pairs for one poll of one pack.

    PIA and PIB are always read. identity adds VIA and the part model,
    status the PIC and SFA coil tables, settings the SPA registers.
    """
    blocks = [("pia", 0x1000, 0x12, 0x04), ("pib", 0x1100, 0x1A, 0x04)]
    if identity:
        blocks += [("via", 0x1700, 0x33, 0x04), ("model", 0x170A, 0x0A, 0x04)]
    if status:
        blocks += [("pic", 0x1200, 0x90, 0x01), ("sfa", 0x1400, 0x50, 0x01)]
    if settings:
        blocks.append(("spa", 0x1300, 0x6A, 0x04))
    return [(block, build_read_command(battery_addr, register, count, function))
            for block, register, count, function in blocks]


def decode_poll_responses(blocks: list, responses: list, battery_addr: int):
    """Decode one pack's poll into (address, pia, pib, via, pic, sfa, spa).

    blocks names the block each response answers, as build_poll_commands
    returned it; a block that was not polled decodes to None.
    """
    raw = dict(zip(blocks, responses))
    via = None
    if raw.get("via"):
        via = decode_via_response(raw["via"], raw.get("model"))
    return (
        f"0x{battery_addr:02X}",
        decode_pia_table(raw["pia"]) if raw.get("pia") else None,
        decode_pib_table(raw["pib"]) if raw.get("pib") else None,
        via,
        decode_pic_response(raw["pic"]) if raw.get("pic") else None,
        decode_sfa_response(raw["sfa"]) if raw.get("sfa") else None,
        decode_spa_response(raw["spa"]) if raw.get("spa") else None,
    )


# ---------------------------------------------------------------------------
# BMS address discovery
# ---------------------------------------------------------------------------

# Valid PIA response: addr(1) + 0x04(1) + byte_count(1=0x24) + data(36) + crc(2) = 41 bytes
_PIA_RESPONSE_BYTES = 41
# Hex chars for a valid PIA response: 41 bytes × 2 = 82 hex chars
_PIA_RESPONSE_HEX_LEN = _PIA_RESPONSE_BYTES * 2


def discover_bms_address(send_fn, port, baudrate=19200, max_addr=0x0F):
    """Scan Modbus addresses 0x00–max_addr to find a BMS.

    Sends a PIA command to each address and validates the response
    (CRC, frame structure, expected byte count).

    Args:
        send_fn:  Callable (commands, port, baudrate) -> [response_hex_str, ...]
        port:     Serial port path (e.g. "/dev/ttyUSB0")
        baudrate: Baud rate
        max_addr: Highest address to scan (default 0x0F)

    Returns:
        (int | None) Found address, or None if no BMS responds.

    """
    _LOGGER.info("Scanning for BMS on %s (addresses 0x00–0x%02X)...", port, max_addr)

    for addr in range(max_addr + 1):
        cmd = build_read_command(addr, 0x1000, 0x12)
        responses = send_fn([cmd], port, baudrate)

        if not responses or not responses[0]:
            _LOGGER.debug("Address 0x%02X: no response", addr)
            continue

        response = responses[0]

        if len(response) < _PIA_RESPONSE_HEX_LEN:
            _LOGGER.debug(
                "Address 0x%02X: response too short (%d chars, expected %d)",
                addr, len(response), _PIA_RESPONSE_HEX_LEN
            )
            continue

        # First byte should match the address we queried
        if not response.startswith(f"{addr:02x}"):
            _LOGGER.debug(
                "Address 0x%02X: response starts with unexpected addr byte 0x%s",
                addr, response[:2]
            )
            continue

        # Function code must be 0x04
        if response[2:4] != "04":
            _LOGGER.debug(
                "Address 0x%02X: unexpected function code 0x%s",
                addr, response[2:4]
            )
            continue

        # Byte count must be 0x24 (36 data bytes for PIA)
        if response[4:6] != "24":
            _LOGGER.debug(
                "Address 0x%02X: unexpected byte_count 0x%s",
                addr, response[4:6]
            )
            continue

        # CRC validation
        if not verify_crc(response):
            _LOGGER.debug("Address 0x%02X: CRC invalid", addr)
            continue

        _LOGGER.info(
            "BMS found at address 0x%02X — valid PIA response (%d chars)",
            addr, len(response)
        )
        return addr

    _LOGGER.warning(
        "No BMS found on %s scanning addresses 0x00–0x%02X",
        port, max_addr
    )
    return None


# ---------------------------------------------------------------------------
# Type conversion
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------

def convert_bytes_to_data(data_type: str, byte1: int, byte2: int):
    """Convert two bytes (MSB, LSB) to a typed value.

    Args:
        data_type: "UINT16" or "INT16"
        byte1:     Most-significant byte.
        byte2:     Least-significant byte.

    """
    if data_type == "UINT16":
        return (byte1 << 8) | byte2
    elif data_type == "INT16":
        value = (byte1 << 8) | byte2
        if value & 0x8000:
            value -= 0x10000
        return value
    return None


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

class V3PIATableData:
    """Pack Info A — general battery pack data."""

    def __init__(self):
        self.pack_voltage = 0                # V
        self.current = 0                     # A  (negative = discharge)
        self.remaining_capacity = 0          # Ah
        self.total_capacity = 0              # Ah
        self.total_discharge_capacity = 0    # Ah  (cumulative)
        self.soc = 0                         # %
        self.soh = 0                         # %
        self.cycle = 0                       # cycles
        self.system_events = 0               # bitfield
        self.avg_cell_voltage = 0            # V
        self.avg_cell_temperature = 0        # °C
        self.max_cell_voltage = 0            # V
        self.min_cell_voltage = 0            # V
        self.max_cell_temperature = 0        # °C
        self.min_cell_temperature = 0        # °C
        self.max_discharge_current = 0       # A
        self.max_charge_current = 0          # A
        self.extern_voltage = 0              # V

    def __str__(self):
        return (
            f"pack_voltage={self.pack_voltage}V, "
            f"current={self.current}A, "
            f"remaining={self.remaining_capacity}Ah, "
            f"total={self.total_capacity}Ah, "
            f"discharge_total={self.total_discharge_capacity}Ah, "
            f"soc={self.soc}%, soh={self.soh}%, cycle={self.cycle}, "
            f"avg_cell_v={self.avg_cell_voltage}V, "
            f"avg_cell_t={self.avg_cell_temperature}°C, "
            f"max_cell_v={self.max_cell_voltage}V, "
            f"min_cell_v={self.min_cell_voltage}V, "
            f"max_cell_t={self.max_cell_temperature}°C, "
            f"min_cell_t={self.min_cell_temperature}°C"
        )


class V3PIBTableData:
    """Pack Info B — cell voltages and temperatures."""

    def __init__(self):
        # Cell voltages (V)
        self.cell1_voltage = 0
        self.cell2_voltage = 0
        self.cell3_voltage = 0
        self.cell4_voltage = 0
        self.cell5_voltage = 0
        self.cell6_voltage = 0
        self.cell7_voltage = 0
        self.cell8_voltage = 0
        self.cell9_voltage = 0
        self.cell10_voltage = 0
        self.cell11_voltage = 0
        self.cell12_voltage = 0
        self.cell13_voltage = 0
        self.cell14_voltage = 0
        self.cell15_voltage = 0
        self.cell16_voltage = 0
        # Cell temperatures (°C)
        self.cell_temperature_1 = 0
        self.cell_temperature_2 = 0
        self.cell_temperature_3 = 0
        self.cell_temperature_4 = 0
        self.cell_temperature_5 = 0
        self.cell_temperature_6 = 0
        self.cell_temperature_7 = 0
        self.cell_temperature_8 = 0
        # Environment temperatures (°C)
        self.environment_temperature = 0
        self.power_temperature = 0

    def __str__(self):
        cells = ", ".join(
            f"cell{i}_v={getattr(self, f'cell{i}_voltage')}V"
            for i in range(1, 17)
        )
        return (
            f"{cells}, "
            f"temp1={self.cell_temperature_1}°C, "
            f"temp2={self.cell_temperature_2}°C, "
            f"temp3={self.cell_temperature_3}°C, "
            f"temp4={self.cell_temperature_4}°C, "
            f"env_temp={self.environment_temperature}°C, "
            f"pwr_temp={self.power_temperature}°C"
        )


# ---------------------------------------------------------------------------
# PIA decoding
# ---------------------------------------------------------------------------

class V3VIA:
    """Version/identification data from the VIA block."""

    def __init__(self):
        self.manufacturer = ""
        self.model = ""
        self.serial = ""


class V3PIC:
    """PIC alarm bits (144), plus the 18 status bytes decoded by name."""

    def __init__(self):
        self.bits = []
        self.raw = b""

    def _byte(self, name):
        offset = PIC_BYTES[name]
        return self.raw[offset] if offset < len(self.raw) else 0

    def __getattr__(self, name):
        # Byte-level status codes, e.g. pic.fet_event_code.
        if name in PIC_BYTES:
            return self._byte(name)
        raise AttributeError(name)

    @property
    def balancing_cells(self) -> list:
        """Cell numbers (1-16) currently equalising."""
        low = self._byte("equalization_cells_01_08")
        high = self._byte("equalization_cells_09_16")
        return [bit + 1 for bit in range(8) if low >> bit & 1] + \
               [bit + 9 for bit in range(8) if high >> bit & 1]

    @property
    def balancing_cell_count(self) -> int:
        return len(self.balancing_cells)

    @property
    def heating(self) -> bool:
        """Pack heater running (FET event code, TB07 bit 3)."""
        return bool(self._byte("fet_event_code") & 0x08)

    @property
    def discharge_fet(self) -> bool:
        return bool(self._byte("fet_event_code") & 0x01)

    @property
    def charge_fet(self) -> bool:
        return bool(self._byte("fet_event_code") & 0x02)

    @property
    def current_limiting(self) -> bool:
        return bool(self._byte("fet_event_code") & 0x04)

    @property
    def charging(self) -> bool:
        return bool(self._byte("system_state_code") & 0x02)

    @property
    def discharging(self) -> bool:
        return bool(self._byte("system_state_code") & 0x01)

    @property
    def system_state(self) -> str:
        """Readable system state (TB09)."""
        names = _set_bits(self._byte("system_state_code"), SYSTEM_STATE_BITS)
        return ", ".join(names) if names else "Unknown"

    @property
    def active_alarms(self) -> list:
        """Names of every alarm and protection currently raised."""
        alarms = []
        for code, bits in ALARM_CODE_BITS:
            alarms += _set_bits(self._byte(code), bits)
        for code, label, first in CELL_ALARM_BYTES:
            value = self._byte(code)
            alarms += [f"{label} {first + bit}" for bit in range(8) if value >> bit & 1]
        return alarms

    @property
    def active_faults(self) -> list:
        """Names of hardware faults currently raised (TB15)."""
        return _set_bits(self._byte("hard_fault_event_code"), HARD_FAULT_BITS)

    @property
    def status_flags(self) -> list:
        """Battery equalisation state code flags (TB08)."""
        return _set_bits(self._byte("battery_equalization_state_code"),
                         EQUALIZATION_STATE_BITS)


# PIC payload byte offset of each status code (0x1200-0x128F, 8 coils a byte).
PIC_BYTES = {
    "cells_01_08_undervoltage_alarm": 0,
    "cells_09_16_undervoltage_alarm": 1,
    "cells_01_08_overvoltage_alarm": 2,
    "cells_09_16_overvoltage_alarm": 3,
    "cell_temperature_low_alarm": 4,
    "cell_temperature_high_alarm": 5,
    "equalization_cells_01_08": 6,
    "equalization_cells_09_16": 7,
    "system_state_code": 8,
    "voltage_event_code": 9,
    "cell_temperature_event_code": 10,
    "environment_power_temperature_event_code": 11,
    "current_event_code_1": 12,
    "current_event_code_2": 13,
    "residual_capacity_code": 14,
    "fet_event_code": 15,
    "battery_equalization_state_code": 16,
    "hard_fault_event_code": 17,
}


def _set_bits(value: int, names: tuple) -> list:
    """Names whose bit is set in value; None entries are reserved bits."""
    return [name for bit, name in enumerate(names) if name and value >> bit & 1]


# Bit tables from the Seplos V3 Modbus RTU protocol, index = bit number.
SYSTEM_STATE_BITS = (  # TB09
    "Discharge", "Charge", "Floating charge", "Full charge", "Standby", "Off",
)
EQUALIZATION_STATE_BITS = (  # TB08
    "Low SoC alarm", "Intermittent charge", "External switch control",
    "Static standby and sleep", "History data recording", "Under SoC protection",
    "Active current limiting", "Passive current limiting",
)
HARD_FAULT_BITS = (  # TB15
    "NTC fault", "AFE fault", "Charge MOSFET fault", "Discharge MOSFET fault",
    "Cell fault", "Break line fault", "Key fault", "Aerosol alarm",
)
ALARM_CODE_BITS = (
    ("voltage_event_code", (  # TB02
        "Cell high voltage alarm", "Cell over voltage protection",
        "Cell low voltage alarm", "Cell under voltage protection",
        "Pack high voltage alarm", "Pack over voltage protection",
        "Pack low voltage alarm", "Pack under voltage protection",
    )),
    ("cell_temperature_event_code", (  # TB03
        "Charge high temperature alarm", "Charge over temperature protection",
        "Charge low temperature alarm", "Charge under temperature protection",
        "Discharge high temperature alarm", "Discharge over temperature protection",
        "Discharge low temperature alarm", "Discharge under temperature protection",
    )),
    # TB04 bit 6 ("Cell temperature low heating") is the heater, not an alarm.
    ("environment_power_temperature_event_code", (
        "High environment temperature alarm",
        "Over environment temperature protection",
        "Low environment temperature alarm",
        "Under environment temperature protection",
        "High power temperature alarm", "Over power temperature protection",
    )),
    ("current_event_code_1", (  # TB05
        "Charge current alarm", "Charge over current protection",
        "Charge second level current protection", "Discharge current alarm",
        "Discharge over current protection",
        "Discharge second level over current protection",
        "Output short circuit protection",
    )),
    ("current_event_code_2", (  # TB16
        "Output short latch up", None, "Second charge latch up",
        "Second discharge latch up",
    )),
    ("residual_capacity_code", (  # TB06
        None, None, "SoC alarm", "SoC protection", "Cell difference alarm",
    )),
)
# Per-cell alarm bytes: (code, label, number of bit 0). The protocol documents
# one temperature byte each, covering sensors 1-8.
CELL_ALARM_BYTES = (
    ("cells_01_08_undervoltage_alarm", "Low voltage cell", 1),
    ("cells_09_16_undervoltage_alarm", "Low voltage cell", 9),
    ("cells_01_08_overvoltage_alarm", "High voltage cell", 1),
    ("cells_09_16_overvoltage_alarm", "High voltage cell", 9),
    ("cell_temperature_low_alarm", "Low temperature sensor", 1),
    ("cell_temperature_high_alarm", "High temperature sensor", 1),
)


class V3SFA:
    """SFA switch-parameter bits (80)."""

    def __init__(self):
        self.bits = []
        self.raw = b""


class V3SPA:
    """SPA integer settings (106 values keyed by register address)."""

    def __init__(self):
        self.values = {}
        self.raw = b""


def _frame_data(response, expected_data_len):
    """Validate and return the data bytes of a Modbus response, or None."""
    if not response:
        return None
    if response.startswith("~"):
        response = response[1:]
    if not verify_crc(response):
        _LOGGER.error("CRC invalid for frame %s", response)
        return None
    try:
        raw = bytes.fromhex(response)
    except ValueError:
        return None
    if len(raw) < 3 + expected_data_len + 2:
        _LOGGER.warning(
            "Frame too short: %d bytes, expected >= %d",
            len(raw), 3 + expected_data_len + 2,
        )
        return None
    return raw[3:-2]


def decode_model_response(response):
    """Decode the 20-byte part-model block."""
    data = _frame_data(response, 0x14)
    if data is None:
        return ""
    return data[:20].split(b"\0")[0].decode("ascii", errors="replace").strip()


def decode_via_response(response, model_response=None):
    """Decode VIA manufacturer/model/serial plus the separate model block."""
    data = _frame_data(response, 0x66)
    if data is None:
        return None
    model = decode_model_response(model_response) if model_response else ""
    combined = data[:40].split(b"\0")[0].decode("ascii", errors="replace").strip()
    serial = data[40:].split(b"\0")[0].decode("ascii", errors="replace").strip()
    via = V3VIA()
    via.serial = serial
    via.model = model
    if model and combined.endswith(model):
        via.manufacturer = combined[: -len(model)].strip()
    else:
        via.manufacturer = combined
    return via


def _decode_bits(response, expected_data_len, spec):
    data = _frame_data(response, expected_data_len)
    if data is None:
        return None
    bits = []
    for item in spec:
        try:
            bits.append((data[item["byte"]] >> item["bit"]) & 1)
        except IndexError:
            bits.append(0)
    return bits, data


def decode_pic_response(response):
    result = _decode_bits(response, 0x12, PIC_BITS)
    if result is None:
        return None
    bits, data = result
    pic = V3PIC()
    pic.bits = bits
    pic.raw = data
    return pic


def decode_sfa_response(response):
    result = _decode_bits(response, 0x0A, SFA_BITS)
    if result is None:
        return None
    bits, data = result
    sfa = V3SFA()
    sfa.bits = bits
    sfa.raw = data
    return sfa


def decode_spa_response(response):
    data = _frame_data(response, 0xD4)
    if data is None:
        return None
    spa = V3SPA()
    spa.raw = data
    for field in SPA_FIELDS:
        off = field["index"]
        try:
            raw = int.from_bytes(data[off:off + 2], "big")
        except Exception:
            continue
        if field["signed"] and raw >= 0x8000:
            raw -= 0x10000
        value = raw * field["scale"]
        if field["unit"] == "°C":
            value -= 273.15
        spa.values[field["addr"]] = value
    return spa


def decode_pia_table(response: str):
    """Decode the Modbus response to a PIA command (registers 0x1000–0x1011).

    Frame layout:
        [addr, 0x04, LEN, DATA..., CRC_lo, CRC_hi]

    Register layout (per Seplos V3 Modbus spec):
        0x1000  pack_voltage             UINT16  10mV    → × 0.01   → V
        0x1001  current                  INT16   10mA    → × 0.01   → A
        0x1002  remaining_capacity       UINT16  10mAh   → × 0.01   → Ah
        0x1003  total_capacity           UINT16  10mAh   → × 0.01   → Ah
        0x1004  total_discharge_capacity UINT16  10Ah    → × 10     → Ah
        0x1005  soc                      UINT16  0.1%    → × 0.1    → %
        0x1006  soh                      UINT16  0.1%    → × 0.1    → %
        0x1007  cycle                    UINT16  1
        0x1008  avg_cell_voltage         UINT16  1mV     → × 0.001  → V
        0x1009  avg_cell_temperature     UINT16  0.1K    → × 0.1 − 273.15 → °C
        0x100A  max_cell_voltage         UINT16  1mV     → × 0.001  → V
        0x100B  min_cell_voltage         UINT16  1mV     → × 0.001  → V
        0x100C  max_cell_temperature     UINT16  0.1K    → × 0.1 − 273.15 → °C
        0x100D  min_cell_temperature     UINT16  0.1K    → × 0.1 − 273.15 → °C
        0x100E  (reserved)
        0x100F  max_discharge_current    UINT16  1A
        0x1010  max_charge_current       UINT16  1A
        0x1011  (reserved)
    """
    if not response:
        _LOGGER.warning("decode_pia_table: empty response")
        return None

    # Strip possible legacy '~' prefix
    if response.startswith("~"):
        response = response[1:]

    # CRC check — reject immediately on failure
    if not verify_crc(response):
        _LOGGER.error("decode_pia_table: CRC invalid for frame %s — discarding", response)
        return None

    try:
        raw = bytes.fromhex(response)
    except ValueError as e:
        _LOGGER.error("decode_pia_table: unable to decode hex frame: %s", e)
        return None

    # Minimum length: 3 header + 36 data + 2 CRC = 41 bytes
    if len(raw) < 41:
        _LOGGER.warning("decode_pia_table: frame too short (%d bytes, expected >= 41)", len(raw))
        return None

    data = raw[3:-2]  # skip header, drop CRC
    pia = V3PIATableData()

    try:
        pia.pack_voltage             = convert_bytes_to_data("UINT16", data[0],  data[1])  * 0.01
        pia.current                  = convert_bytes_to_data("INT16",  data[2],  data[3])  * 0.01
        pia.remaining_capacity       = convert_bytes_to_data("UINT16", data[4],  data[5])  * 0.01
        pia.total_capacity           = convert_bytes_to_data("UINT16", data[6],  data[7])  * 0.01
        # Spec says unit = 10Ah for total_discharge_capacity
        pia.total_discharge_capacity = convert_bytes_to_data("UINT16", data[8],  data[9])  * 10
        pia.soc                      = convert_bytes_to_data("UINT16", data[10], data[11]) * 0.1
        pia.soh                      = convert_bytes_to_data("UINT16", data[12], data[13]) * 0.1
        pia.cycle                    = convert_bytes_to_data("UINT16", data[14], data[15])
        pia.avg_cell_voltage         = convert_bytes_to_data("UINT16", data[16], data[17]) * 0.001
        pia.avg_cell_temperature     = convert_bytes_to_data("UINT16", data[18], data[19]) * 0.1 - 273.15
        pia.max_cell_voltage         = convert_bytes_to_data("UINT16", data[20], data[21]) * 0.001
        pia.min_cell_voltage         = convert_bytes_to_data("UINT16", data[22], data[23]) * 0.001
        pia.max_cell_temperature     = convert_bytes_to_data("UINT16", data[24], data[25]) * 0.1 - 273.15
        pia.min_cell_temperature     = convert_bytes_to_data("UINT16", data[26], data[27]) * 0.1 - 273.15
        pia.system_events            = convert_bytes_to_data("UINT16", data[28], data[29])
        pia.max_discharge_current    = convert_bytes_to_data("UINT16", data[30], data[31])
        pia.max_charge_current       = convert_bytes_to_data("UINT16", data[32], data[33])
        pia.extern_voltage           = convert_bytes_to_data("UINT16", data[34], data[35]) * 0.001
    except (IndexError, TypeError) as e:
        _LOGGER.error("decode_pia_table: index/type error: %s", e)
        return None

    _LOGGER.debug("PIA decoded: %s", pia)
    return pia


# ---------------------------------------------------------------------------
# PIB decoding
# ---------------------------------------------------------------------------

def decode_pib_table(response: str):
    """Decode the Modbus response to a PIB command (registers 0x1100–0x1119).

    Frame layout:
        [addr, 0x04, LEN, DATA..., CRC_lo, CRC_hi]

    Register layout:
        0x1100–0x110F  cell voltages 1–16       UINT16  1mV    → × 0.001 → V
        0x1110–0x1113  cell temperatures 1–4     UINT16  0.1K   → × 0.1 − 273.15 → °C
        0x1114–0x1117  (reserved)
        0x1118         environment temperature   UINT16  0.1K   → × 0.1 − 273.15 → °C
        0x1119         power temperature          UINT16  0.1K   → × 0.1 − 273.15 → °C
    """
    if not response:
        _LOGGER.warning("decode_pib_table: empty response")
        return None

    if response.startswith("~"):
        response = response[1:]

    # CRC check — reject immediately on failure
    if not verify_crc(response):
        _LOGGER.error("decode_pib_table: CRC invalid for frame %s — discarding", response)
        return None

    try:
        raw = bytes.fromhex(response)
    except ValueError as e:
        _LOGGER.error("decode_pib_table: unable to decode hex frame: %s", e)
        return None

    # Minimum length: 3 header + 52 data + 2 CRC = 57 bytes
    if len(raw) < 57:
        _LOGGER.warning("decode_pib_table: frame too short (%d bytes, expected >= 57)", len(raw))
        return None

    data = raw[3:-2]
    pib = V3PIBTableData()

    try:
        # Cell voltages (registers 0x1100–0x110F, 16 cells)
        pib.cell1_voltage  = convert_bytes_to_data("UINT16", data[0],  data[1])  * 0.001
        pib.cell2_voltage  = convert_bytes_to_data("UINT16", data[2],  data[3])  * 0.001
        pib.cell3_voltage  = convert_bytes_to_data("UINT16", data[4],  data[5])  * 0.001
        pib.cell4_voltage  = convert_bytes_to_data("UINT16", data[6],  data[7])  * 0.001
        pib.cell5_voltage  = convert_bytes_to_data("UINT16", data[8],  data[9])  * 0.001
        pib.cell6_voltage  = convert_bytes_to_data("UINT16", data[10], data[11]) * 0.001
        pib.cell7_voltage  = convert_bytes_to_data("UINT16", data[12], data[13]) * 0.001
        pib.cell8_voltage  = convert_bytes_to_data("UINT16", data[14], data[15]) * 0.001
        pib.cell9_voltage  = convert_bytes_to_data("UINT16", data[16], data[17]) * 0.001
        pib.cell10_voltage = convert_bytes_to_data("UINT16", data[18], data[19]) * 0.001
        pib.cell11_voltage = convert_bytes_to_data("UINT16", data[20], data[21]) * 0.001
        pib.cell12_voltage = convert_bytes_to_data("UINT16", data[22], data[23]) * 0.001
        pib.cell13_voltage = convert_bytes_to_data("UINT16", data[24], data[25]) * 0.001
        pib.cell14_voltage = convert_bytes_to_data("UINT16", data[26], data[27]) * 0.001
        pib.cell15_voltage = convert_bytes_to_data("UINT16", data[28], data[29]) * 0.001
        pib.cell16_voltage = convert_bytes_to_data("UINT16", data[30], data[31]) * 0.001

        # Cell temperatures (registers 0x1110–0x1113)
        pib.cell_temperature_1 = convert_bytes_to_data("UINT16", data[32], data[33]) * 0.1 - 273.15
        pib.cell_temperature_2 = convert_bytes_to_data("UINT16", data[34], data[35]) * 0.1 - 273.15
        pib.cell_temperature_3 = convert_bytes_to_data("UINT16", data[36], data[37]) * 0.1 - 273.15
        pib.cell_temperature_4 = convert_bytes_to_data("UINT16", data[38], data[39]) * 0.1 - 273.15

        # Cell temperatures 5–8 (registers 0x1114–0x1117)
        pib.cell_temperature_5 = convert_bytes_to_data("UINT16", data[40], data[41]) * 0.1 - 273.15
        pib.cell_temperature_6 = convert_bytes_to_data("UINT16", data[42], data[43]) * 0.1 - 273.15
        pib.cell_temperature_7 = convert_bytes_to_data("UINT16", data[44], data[45]) * 0.1 - 273.15
        pib.cell_temperature_8 = convert_bytes_to_data("UINT16", data[46], data[47]) * 0.1 - 273.15

        # Environment temperature (register 0x1118)
        if len(data) >= 50:
            pib.environment_temperature = convert_bytes_to_data("UINT16", data[48], data[49]) * 0.1 - 273.15

        # Power temperature (register 0x1119)
        if len(data) >= 52:
            pib.power_temperature = convert_bytes_to_data("UINT16", data[50], data[51]) * 0.1 - 273.15

    except (IndexError, TypeError) as e:
        _LOGGER.error("decode_pib_table: index/type error during decoding: %s", e)
        return None

    _LOGGER.debug("PIB decoded: %s", pib)
    return pib


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def extract_data_from_message(msg, telemetry_requested=True, teledata_requested=True,
                               debug=True, config_battery_address=None):
    """Decode PIA, PIB, VIA/model, PIC, SFA and SPA responses."""
    if isinstance(config_battery_address, int):
        address_string = f"0x{config_battery_address:02X}"
    else:
        address_string = str(config_battery_address) if config_battery_address else "unknown"

    pia = pib = via = pic = sfa = spa = None
    if not msg:
        _LOGGER.error("extract_data_from_message: no responses")
        return address_string, pia, pib, via, pic, sfa, spa

    def get(idx):
        if len(msg) <= idx:
            return ""
        value = msg[idx]
        return value[1:] if isinstance(value, str) and value.startswith("~") else value

    if len(msg) > 0:
        pia = decode_pia_table(get(0))
    if len(msg) > 1:
        pib = decode_pib_table(get(1))
    if len(msg) > 2:
        via = decode_via_response(get(2), get(3))
    if len(msg) > 4:
        pic = decode_pic_response(get(4))
    if len(msg) > 5:
        sfa = decode_sfa_response(get(5))
    if len(msg) > 6:
        spa = decode_spa_response(get(6))

    return address_string, pia, pib, via, pic, sfa, spa
