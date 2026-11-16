from homeassistant.components.sensor import SensorEntity, SensorDeviceClass, SensorStateClass
from homeassistant.const import EntityCategory
from homeassistant.helpers.update_coordinator import CoordinatorEntity
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator
from homeassistant.helpers.entity import DeviceInfo

from .data_parser import (
    build_poll_commands,
    decode_poll_responses,
    discover_bms_address,
)
from ....connector.local_serial.seplos_v3_local_serial import send_serial_command as v3_send_serial_command
from ....connector.local_serial.seplos_v3_local_serial import send_telnet_command as v3_send_telnet_command
import functools
import logging
import time
from datetime import timedelta
from .const import (
    ALARM_MAPPINGS,
)
from .vendor_spec import SPA_FIELDS
from ....const import DOMAIN

_LOGGER = logging.getLogger(__name__)

# Sentinel pour distinguer "attribut absent" (→ chercher dans l'objet suivant)
# de "attribut présent mais valant 0" (valeur numérique valide à retourner)
_MISSING = object()

# Per-attempt read budget while scanning for an unknown BMS address.
_DISCOVERY_TIMEOUT = 2

# Protection settings rarely change; re-read them this often (seconds).
_SETTINGS_REFRESH = 3600

# Index of each decoded block in a pack's data tuple.
_PIA, _PIB, _VIA, _PIC, _SFA, _SPA = 1, 2, 3, 4, 5, 6


def parse_battery_addresses(config_battery_address):
    """Parse the configured battery address(es) into a list of ints.

    Accepts a single address ("1", "0x01", 1) or a comma-separated list
    ("1,2,3"). Duplicates are dropped, order is preserved, and unparseable
    entries are skipped with a warning. Falls back to [1] if nothing usable
    remains.
    """
    if isinstance(config_battery_address, (list, tuple)):
        raw_parts = [str(p) for p in config_battery_address]
    else:
        raw_parts = str(config_battery_address if config_battery_address is not None
                        else "1").split(",")

    addresses = []
    for part in raw_parts:
        part = part.strip()
        if not part:
            continue
        try:
            # Decimal unless explicitly hex. int(part, 0) would reject the
            # zero-padded forms people naturally type, such as "01,02,03".
            if part.lower().startswith(("0x", "-0x")):
                value = int(part, 16)
            else:
                value = int(part, 10)
        except ValueError:
            _LOGGER.warning("Battery address '%s' invalid, ignoring", part)
            continue
        if value not in addresses:
            addresses.append(value)

    if not addresses:
        _LOGGER.warning(
            "No usable battery address in '%s', falling back to address 1",
            config_battery_address,
        )
        addresses = [1]

    return addresses


def seplos_v3_device_info(entry_id, sensor_prefix, modbus_address, multi_pack,
                          settings=False):
    """Device for a V3 pack entity.

    One device per pack when the entry covers several addresses, otherwise a
    single device for the entry. Settings entities get a sibling device.
    """
    if multi_pack:
        key = f"seplos_v3_{entry_id}_{modbus_address}"
        name = f"{sensor_prefix} 0x{modbus_address:02X}"
    else:
        key = f"seplos_v3_{entry_id}"
        name = f"{sensor_prefix}"
    if settings:
        key += "_settings"
        name += " Settings"
    return DeviceInfo(
        identifiers={("bms_connector", key)},
        name=name,
        manufacturer="Seplos",
        model="V3 Settings" if settings else "V3 BMS",
        sw_version="Unknown",
    )


def pack_data(coordinator, modbus_address):
    """Return a pack's decoded data tuple from the coordinator, or None."""
    data = coordinator.data
    if isinstance(data, dict):
        return data.get(modbus_address)
    return None


# ---------------------------------------------------------------------------
# generate_sensors
# ---------------------------------------------------------------------------

async def generate_sensors(hass, bms_type, connector_info, config_battery_address,
                            sensor_prefix, entry_id, async_add_entities, poll_interval=10,
                            read_status_flags=True, read_settings=True):
    """Génère et enregistre les capteurs pour une ou plusieurs batteries.

    config_battery_address accepte une adresse unique ("1", "0x01", 1) ou une
    liste séparée par des virgules ("1,2,3"). Toutes les adresses partagent un
    seul coordinator et un seul appel transport par cycle ; chaque pack devient
    alors un device.

    read_status_flags ajoute les tables PIC et SFA, read_settings la table SPA.
    Les blocs d'identification (VIA, modèle) ne sont lus qu'une fois par pack,
    les réglages SPA au plus une fois par _SETTINGS_REFRESH.
    """

    addresses = parse_battery_addresses(config_battery_address)
    multi_pack = len(addresses) > 1
    # VIA and model are static: read until a pack has answered them once.
    identity = {}
    # SPA settings per pack, with the monotonic time they were read.
    settings_cache = {}

    connector_type = connector_info.get("type", "serial")
    serial_port = connector_info.get("port")
    serial_baud = connector_info.get("baudrate", 19200)
    telnet_host = connector_info.get("host")
    telnet_port = connector_info.get("port", 23)
    telnet_timeout = connector_info.get("timeout", 8)

    async def send(commands):
        if connector_type == "telnet":
            return await hass.async_add_executor_job(
                v3_send_telnet_command, commands, telnet_host, telnet_port, telnet_timeout
            )
        return await hass.async_add_executor_job(
            v3_send_serial_command, commands, serial_port, serial_baud
        )

    async def poll(active_addresses):
        plan = []
        for addr in active_addresses:
            blocks = build_poll_commands(
                addr, identity=addr not in identity,
                status=read_status_flags,
                settings=read_settings and (
                    addr not in settings_cache
                    or time.monotonic() - settings_cache[addr][1] > _SETTINGS_REFRESH
                ),
            )
            plan.append((addr, blocks))
        commands = [cmd for _, blocks in plan for _, cmd in blocks]
        _LOGGER.debug(
            "Polling %d battery address(es) %s using %d commands",
            len(active_addresses), [hex(a) for a in active_addresses], len(commands)
        )
        responses = list(await send(commands) or [])
        responses += [""] * (len(commands) - len(responses))
        return plan, responses

    # ------------------------------------------------------------------
    # Fonction de mise à jour
    # ------------------------------------------------------------------

    async def async_update_data():
        """Interroge les batteries via le bus RS485 avec leur adresse Modbus
        propre, en un seul appel transport, puis parse les réponses.

        IMPORTANT : on n'utilise PAS l'adresse 0x00 (broadcast) qui ferait
        répondre toutes les batteries simultanément et créerait des collisions
        sur le bus RS485.
        """
        active_addresses = list(addresses)
        plan, responses = await poll(active_addresses)

        # Auto-découverte d'adresse si la configurée ne répond pas, uniquement
        # quand une seule adresse est configurée.
        if len(active_addresses) == 1 and not responses[0]:
            addr_int = active_addresses[0]
            _LOGGER.warning(
                "No response from configured address 0x%02X — scanning for BMS...",
                addr_int
            )
            # discover_bms_address calls send_fn(cmds, arg2, arg3) positionally,
            # matching send_telnet_command(commands, host, port) and
            # send_serial_command(commands, port, baudrate).
            #
            # The scan tries every address in turn, and over TCP each attempt is
            # its own connection. Cap the per-attempt read so a silent bus costs
            # seconds rather than minutes of an executor thread.
            if connector_type == "telnet":
                scan_fn = functools.partial(v3_send_telnet_command,
                                            timeout=_DISCOVERY_TIMEOUT)
                discovered = await hass.async_add_executor_job(
                    discover_bms_address, scan_fn, telnet_host, telnet_port
                )
            else:
                discovered = await hass.async_add_executor_job(
                    discover_bms_address, v3_send_serial_command,
                    serial_port, serial_baud
                )
            if discovered is not None and discovered != addr_int:
                _LOGGER.warning(
                    "BMS found at address 0x%02X (configured was 0x%02X) — "
                    "auto-using 0x%02X for this session. Consider updating your "
                    "configuration to address 0x%02X.",
                    discovered, addr_int, discovered, discovered
                )
                plan, responses = await poll([discovered])
                # Entities are keyed by the configured address.
                plan = [(addr_int, plan[0][1])]
            elif discovered is None:
                _LOGGER.error(
                    "No BMS found on %s — check wiring and BMS address",
                    telnet_host or serial_port or "unknown"
                )

        results = {}
        offset = 0
        for addr, blocks in plan:
            chunk = responses[offset:offset + len(blocks)]
            offset += len(blocks)
            if not chunk[0]:
                _LOGGER.warning("No valid response from battery 0x%02X", addr)
                results[addr] = (f"0x{addr:02X}", None, None, identity.get(addr),
                                 None, None, settings_cache.get(addr, (None,))[0])
                continue
            data = decode_poll_responses([b for b, _ in blocks], chunk, addr)
            if data[_VIA] is not None:
                identity[addr] = data[_VIA]
            if data[_SPA] is not None:
                settings_cache[addr] = (data[_SPA], time.monotonic())
            results[addr] = (data[:_VIA] + (identity.get(addr),) + data[_VIA + 1:_SPA]
                             + (settings_cache.get(addr, (None,))[0],))

        return results

    # ------------------------------------------------------------------
    # Premier appel pour initialiser le coordinator
    # ------------------------------------------------------------------

    coordinator = DataUpdateCoordinator(
        hass,
        _LOGGER,
        name=f"seplos_bms_sensor_{config_battery_address}",
        update_method=async_update_data,
        update_interval=timedelta(seconds=poll_interval),
    )

    _LOGGER.debug("async_refresh data generate_sensors called (addr=%s)", config_battery_address)
    await coordinator.async_refresh()

    context = {
        "coordinator": coordinator,
        "addresses": addresses,
        "sensor_prefix": sensor_prefix,
        "multi_pack": multi_pack,
        "read_status_flags": read_status_flags,
    }
    hass.data.setdefault(DOMAIN, {}).setdefault("coordinators", {})[entry_id] = context
    # If the binary_sensor platform set up first, hand it the coordinator now.
    pending_binary = hass.data.get(DOMAIN, {}).get("binary_add", {}).pop(entry_id, None)
    if pending_binary is not None:
        from ....binary_sensor import build_entities
        pending_binary(build_entities(context, entry_id), True)

    sensors = []
    for _mb in addresses:
        common = {
            "battery_address": f"0x{_mb:02X}", "sensor_prefix": sensor_prefix,
            "entry_id": entry_id, "modbus_address": _mb, "multi_pack": multi_pack,
        }

        def sensor(attribute, name, unit, icon, **kwargs):
            return SeplosBMSSensorBase(coordinator, connector_info, attribute, name,
                                       unit, icon, **common, **kwargs)

        # --------------------------------------------------------------
        # PIA (pack global) et VIA (identification)
        # --------------------------------------------------------------
        sensors += [
            sensor("pack_voltage", "Pack Voltage", "V", "mdi:flash-circle"),
            sensor("current", "Current", "A", "mdi:current-ac"),
            sensor("remaining_capacity", "Remaining Capacity", "Ah",
                   "mdi:battery-charging-wireless"),
            sensor("total_capacity", "Total Capacity", "Ah", "mdi:battery"),
            sensor("total_discharge_capacity", "Total Discharge Capacity", "Ah",
                   "mdi:battery-discharging"),
            sensor("soc", "State of Charge", "%", "mdi:gauge"),
            sensor("soh", "State of Health", "%", "mdi:gauge"),
            sensor("cycle", "Cycle", None, "mdi:numeric"),
            sensor("manufacturer", "Manufacturer", None, "mdi:factory"),
            sensor("model", "Model", None, "mdi:information-outline"),
            sensor("serial", "Serial Number", None, "mdi:identifier"),
            sensor("avg_cell_voltage", "Avg Cell Voltage", "V", "mdi:battery-20"),
            sensor("avg_cell_temperature", "Avg Cell Temperature", "°C", "mdi:thermometer"),
            sensor("max_cell_voltage", "Max Cell Voltage", "V", "mdi:battery-high"),
            sensor("min_cell_voltage", "Min Cell Voltage", "V", "mdi:battery-low"),
            sensor("max_cell_temperature", "Max Cell Temperature", "°C",
                   "mdi:thermometer-chevron-up"),
            sensor("min_cell_temperature", "Min Cell Temperature", "°C",
                   "mdi:thermometer-chevron-down"),
            sensor("system_events", "System Events", None, "mdi:list-status"),
            sensor("extern_voltage", "External Voltage", "V", "mdi:flash"),
            sensor("max_discharge_current", "Max Discharge Current", "A", "mdi:current-dc"),
            sensor("max_charge_current", "Max Charge Current", "A", "mdi:current-dc"),
        ]

        # --------------------------------------------------------------
        # PIB (cellules individuelles)
        # --------------------------------------------------------------
        sensors += [
            sensor(f"cell{i}_voltage", f"Cell {i} Voltage", "V", "mdi:battery")
            for i in range(1, 17)
        ] + [
            sensor(f"cell_temperature_{i}", f"Cell Temperature {i}", "°C", "mdi:thermometer")
            for i in range(1, 9)
        ] + [
            sensor("environment_temperature", "Environment Temperature", "°C",
                   "mdi:thermometer-lines"),
            sensor("power_temperature", "Power Temperature", "°C", "mdi:thermometer-lines"),
        ]

        # --------------------------------------------------------------
        # PIC (états et alarmes)
        # --------------------------------------------------------------
        if read_status_flags:
            sensors += [
                BalancingCellsSensor(
                    coordinator, connector_info, "balancing_cell_count",
                    "Balancing Cells", None, "mdi:scale-balance",
                    diagnostic=True, **common,
                ),
                SystemStateSensor(
                    coordinator, connector_info, "system_state",
                    "System State", None, "mdi:battery-sync",
                    classify=False, **common,
                ),
            ] + [
                sensor(attribute, name, None, icon, diagnostic=True, classify=False)
                for attribute, name, icon in (
                    ("system_state_code", "System State Code", "mdi:state-machine"),
                    ("fet_event_code", "FET State Code", "mdi:electric-switch"),
                    ("voltage_event_code", "Voltage Event Code", "mdi:alert-circle-outline"),
                    ("cell_temperature_event_code", "Cell Temperature Event Code",
                     "mdi:alert-circle-outline"),
                    ("environment_power_temperature_event_code",
                     "Environment/Power Temperature Event Code", "mdi:alert-circle-outline"),
                    ("current_event_code_1", "Current Event Code 1", "mdi:alert-circle-outline"),
                    ("current_event_code_2", "Current Event Code 2", "mdi:alert-circle-outline"),
                    ("residual_capacity_code", "Residual Capacity Code", "mdi:battery-alert"),
                    ("hard_fault_event_code", "Hard Fault Event Code", "mdi:alert-octagon"),
                )
            ]

        # --------------------------------------------------------------
        # SPA (réglages de protection)
        # --------------------------------------------------------------
        if read_settings:
            sensors += [
                SeplosV3SettingSensor(
                    coordinator, connector_info, field["addr"], field["name"],
                    field["unit"], **common,
                )
                for field in SPA_FIELDS
            ]

    async_add_entities(sensors, True)


# ---------------------------------------------------------------------------
# Classe de base des capteurs
# ---------------------------------------------------------------------------

class SeplosBMSSensorBase(CoordinatorEntity, SensorEntity):
    """Capteur de base pour un attribut d'une batterie SEPLOS V3."""

    def interpret_alarm(self, event, value):
        flags = ALARM_MAPPINGS.get(event, [])
        if not flags:
            return f"Unknown event: {event}"
        triggered_alarms = [
            flag for idx, flag in enumerate(flags)
            if value is not None and value & (1 << idx)
        ]
        return ', '.join(triggered_alarms) if triggered_alarms else "No Alarm"

    def __init__(self, coordinator, port, attribute, name, unit=None,
                 icon=None, battery_address=None, sensor_prefix=None, entry_id=None,
                 settings=False, modbus_address=None, multi_pack=False,
                 diagnostic=False, classify=True):
        super().__init__(coordinator)
        if diagnostic:
            self._attr_entity_category = EntityCategory.DIAGNOSTIC
        # Entity name already includes prefix; prevent HA 2024+ from doubling.
        self._attr_has_entity_name = False
        self._port = port
        self._attribute = attribute
        self._name = name
        self._unit = unit
        self._icon = icon
        self._battery_address = battery_address
        self._sensor_prefix = sensor_prefix
        # Bitmask codes must not be classified: _set_sensor_attributes matches on
        # substrings, so names like "Voltage Event Code" would otherwise be
        # published as unit-less voltage measurements.
        if classify:
            self._set_sensor_attributes(attribute)
        self._entry_id = entry_id
        self._settings = settings
        # Modbus address this sensor reads from; keys the coordinator payload.
        self._modbus_address = modbus_address
        self._multi_pack = multi_pack

        self._attr_device_info = seplos_v3_device_info(
            entry_id, sensor_prefix, modbus_address, multi_pack, settings
        )

    def _set_sensor_attributes(self, attribute):
        """Set device class and state class based on sensor type."""
        # For derived sensors, attribute is None - check the display name instead
        check = attribute.lower() if attribute else self._name.lower()

        if 'temperature' in check:
            self._attr_device_class = SensorDeviceClass.TEMPERATURE
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'voltage' in check:
            self._attr_device_class = SensorDeviceClass.VOLTAGE
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'current' in check and 'alarm' not in check:
            self._attr_device_class = SensorDeviceClass.CURRENT
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'power' in check or 'watts' in check:
            self._attr_device_class = SensorDeviceClass.POWER
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'soc' in check:
            self._attr_device_class = SensorDeviceClass.BATTERY
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'capacity' in check and 'watts' not in check:
            # Match HEH: capacity sensors need MEASUREMENT state class
            self._attr_state_class = SensorStateClass.MEASUREMENT
        elif 'cycles' in check:
            self._attr_state_class = SensorStateClass.TOTAL_INCREASING

    @property
    def name(self):
        prefix = f"{self._sensor_prefix} - {self._battery_address} -"
        return f"{prefix} {self._name}"

    @property
    def unique_id(self):
        prefix = "settings_" if self._settings else ""
        # The Modbus address is only part of the id when an entry covers
        # several packs.
        if self._multi_pack:
            return (f"bms_connector_v3_{self._entry_id}_{self._modbus_address}_"
                    f"{prefix}{self._name}")
        return f"bms_connector_v3_{self._entry_id}_{prefix}{self._name}"

    def _pack_data(self):
        """Return this sensor's pack data tuple, or None."""
        return pack_data(self.coordinator, self._modbus_address)

    @property
    def state(self):
        if not self._attribute:
            return super().state

        value = _MISSING

        data = self._pack_data()
        if data is not None:
            # Cherche dans PIA, puis PIB, puis VIA/PIC/SPA
            for data_obj in (data[_PIA], data[_PIB], data[_VIA], data[_PIC], data[_SPA]):
                result = self.get_value(data_obj)
                if result is not _MISSING:
                    value = result
                    break

        if value is _MISSING or value is None or value == '':
            if self._attribute == 'current':
                _LOGGER.debug(
                    "current is None, returning 0.00 to avoid 'unknown' in HA"
                )
                return 0.00
            _LOGGER.warning("No data found for %s", self._name)
            return None

        _LOGGER.debug("Sensor state %s: %s", self._name, value)
        return value

    @property
    def unit_of_measurement(self):
        return self._unit

    def get_value(self, data_object):
        """Récupère la valeur d'un attribut depuis un objet de données.

        Retourne _MISSING si l'objet est None ou si l'attribut n'existe pas
        dans cet objet — ce qui permet à state() de continuer à chercher
        dans l'objet suivant (PIA → PIB → ...).

        Retourne la valeur réelle (y compris 0, 0.0, False) si l'attribut
        existe, afin de ne pas confondre "zéro" avec "absent".
        """
        if data_object is None:
            return _MISSING

        # Accès à une liste par index : ex "cell_voltages[0]"
        if '[' in self._attribute and ']' in self._attribute:
            attr, index_str = self._attribute.split('[')
            index = int(index_str.rstrip(']'))
            list_data = getattr(data_object, attr, _MISSING)
            if list_data is _MISSING:
                return _MISSING
            if index < len(list_data):
                return list_data[index]
            return _MISSING

        # Vérifier si l'attribut existe dans cet objet précis
        if not hasattr(data_object, self._attribute):
            return _MISSING

        return getattr(data_object, self._attribute)

    @property
    def icon(self):
        return self._icon


class SeplosV3SettingSensor(SeplosBMSSensorBase):
    """Read-only SPA integer setting on the settings device."""

    def __init__(self, coordinator, port, address, name, unit=None, **kwargs):
        self._address = address
        super().__init__(
            coordinator, port, attribute=None, name=name, unit=unit,
            icon="mdi:tune", settings=True, **kwargs,
        )

    def _set_sensor_attributes(self, attribute):
        unit = self._unit or ""
        if unit == "V":
            self._attr_device_class = SensorDeviceClass.VOLTAGE
        elif unit in ("A", "mA"):
            self._attr_device_class = SensorDeviceClass.CURRENT
        elif unit == "°C":
            self._attr_device_class = SensorDeviceClass.TEMPERATURE
        elif unit in ("s", "ms", "min", "h", "µs"):
            self._attr_device_class = SensorDeviceClass.DURATION
        if unit:
            self._attr_state_class = SensorStateClass.MEASUREMENT

    @property
    def state(self):
        data = self._pack_data()
        if data is not None and data[_SPA] is not None:
            return data[_SPA].values.get(self._address)
        return None


class BalancingCellsSensor(SeplosBMSSensorBase):
    """Number of cells currently equalising, with the cell list as an attribute."""

    @property
    def extra_state_attributes(self):
        data = self._pack_data()
        if data is None or data[_PIC] is None:
            return None
        return {"balancing_cells": data[_PIC].balancing_cells}


class SystemStateSensor(SeplosBMSSensorBase):
    """Readable pack state, with the equalisation state flags as an attribute."""

    @property
    def extra_state_attributes(self):
        data = self._pack_data()
        if data is None or data[_PIC] is None:
            return None
        return {"status_flags": data[_PIC].status_flags}
