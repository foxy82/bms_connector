from homeassistant.components.sensor import SensorEntity, SensorDeviceClass, SensorStateClass
from homeassistant.helpers.update_coordinator import CoordinatorEntity
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator
from homeassistant.helpers.entity import DeviceInfo

from .data_parser import extract_data_from_message, build_commands_for_address, discover_bms_address
from ....connector.local_serial.seplos_v3_local_serial import send_serial_command as v3_send_serial_command
from ....connector.local_serial.seplos_v3_local_serial import send_telnet_command as v3_send_telnet_command
import logging
from datetime import timedelta
from .const import (
    ALARM_MAPPINGS,
)

_LOGGER = logging.getLogger(__name__)

# Sentinel pour distinguer "attribut absent" (→ chercher dans l'objet suivant)
# de "attribut présent mais valant 0" (valeur numérique valide à retourner)
_MISSING = object()


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
            value = int(part, 0)          # accepts "1", "0x01", "03"
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


# ---------------------------------------------------------------------------
# generate_sensors
# ---------------------------------------------------------------------------

async def generate_sensors(hass, bms_type, connector_info, config_battery_address,
                            sensor_prefix, entry_id, async_add_entities, poll_interval=10):
    """Génère et enregistre les capteurs pour une ou plusieurs batteries.

    config_battery_address accepte une adresse unique ("1", "0x01", 1) ou une
    liste séparée par des virgules ("1,2,3"). Toutes les adresses partagent un
    seul coordinator et un seul appel transport par cycle.

    En mono-adresse, le coordinator renvoie le tuple habituel et les unique_id
    gardent leur forme sans l'adresse. En multi-adresse, il renvoie un
    dictionnaire indexé par adresse Modbus et chaque pack devient un device.
    """

    addresses = parse_battery_addresses(config_battery_address)
    multi_pack = len(addresses) > 1

    # ------------------------------------------------------------------
    # Classe dérivée pour les capteurs calculés
    # ------------------------------------------------------------------

    class DerivedSeplosBMSSensor(SeplosBMSSensorBase):
        def __init__(self, *args, **kwargs):
            self._calc_function = kwargs.pop("calc_function", None)
            super().__init__(*args, **kwargs)
            self._config_battery_address = config_battery_address

        @property
        def state(self):
            if self._calc_function:
                result = self._calc_function(self.coordinator.data)
                _LOGGER.debug("Derived sensor '%s' calculated value: %s", self._name, result)
                return result
            return super().state

    # ------------------------------------------------------------------
    # Fonction de mise à jour
    # ------------------------------------------------------------------

    async def async_update_data():
        """Interroge la batterie via le bus RS485 en utilisant son adresse Modbus
        propre (config_battery_address), puis parse les réponses.

        IMPORTANT : on n'utilise PAS l'adresse 0x00 (broadcast) qui ferait
        répondre toutes les batteries simultanément et créerait des collisions
        sur le bus RS485.
        """
        active_addresses = list(addresses)
        addr_int = active_addresses[0]

        # Un seul appel transport pour toutes les adresses : le connecteur
        # réutilise une connexion pour la liste de commandes, donc tous les
        # packs sont échantillonnés dans la même fenêtre.
        commands = []
        for addr in active_addresses:
            commands.extend(build_commands_for_address(addr))
        _LOGGER.debug(
            "Polling %d battery address(es) %s using %d commands",
            len(active_addresses), [hex(a) for a in active_addresses], len(commands)
        )

        # Envoi — utilise le module V3 spécialisé pour Modbus RTU
        # (envoi en binaire brut, pas d'ASCII)
        connector_type = connector_info.get("type", "serial")
        serial_port = connector_info.get("port")
        serial_baud = connector_info.get("baudrate", 19200)

        if connector_type == "telnet":
            telnet_host = connector_info.get("host")
            telnet_port = connector_info.get("port", 23)
            telnet_timeout = connector_info.get("timeout", 8)
            telemetry_data_str = await hass.async_add_executor_job(
                v3_send_telnet_command, commands, telnet_host, telnet_port, telnet_timeout
            )
        else:
            telemetry_data_str = await hass.async_add_executor_job(
                v3_send_serial_command, commands, serial_port, serial_baud
            )

        # Auto-découverte d'adresse si la configurée ne répond pas, uniquement
        # quand une seule adresse est configurée.
        if len(active_addresses) == 1 and (not telemetry_data_str or not telemetry_data_str[0]):
            _LOGGER.warning(
                "No response from configured address 0x%02X — scanning for BMS...",
                addr_int
            )
            discovered = await hass.async_add_executor_job(
                discover_bms_address, v3_send_serial_command, serial_port, serial_baud
            )
            if discovered is not None and discovered != addr_int:
                _LOGGER.warning(
                    "BMS found at address 0x%02X (configured was 0x%02X) — "
                    "auto-using 0x%02X for this session. Consider updating your "
                    "configuration to address 0x%02X.",
                    discovered, addr_int, discovered, discovered
                )
                addr_int = discovered
                active_addresses = [discovered]
                # Réessayer avec la bonne adresse
                commands = build_commands_for_address(addr_int)
                if connector_type == "telnet":
                    telemetry_data_str = await hass.async_add_executor_job(
                        v3_send_telnet_command, commands, telnet_host, telnet_port, telnet_timeout
                    )
                else:
                    telemetry_data_str = await hass.async_add_executor_job(
                        v3_send_serial_command, commands, serial_port, serial_baud
                    )
            elif discovered is None:
                _LOGGER.error(
                    "No BMS found on %s — check wiring and BMS address",
                    serial_port or "unknown"
                )

        telemetry_data_str = telemetry_data_str or []

        # Parsing des réponses : deux commandes (PIA + PIB) par adresse, dans
        # l'ordre d'envoi, donc la paire de l'adresse i est à l'offset i*2.
        results = {}
        for index, addr in enumerate(active_addresses):
            pair = telemetry_data_str[index * 2:index * 2 + 2]
            if len(pair) < 2 or not pair[0]:
                _LOGGER.warning("No valid response from battery 0x%02X", addr)
                results[addr] = (None, None, None, None, None)
                continue
            results[addr] = extract_data_from_message(
                pair,
                telemetry_requested=True,
                teledata_requested=True,
                debug=True,
                config_battery_address=addr,
            )

        # Mono-adresse : renvoie le tuple directement, multi-adresse : le
        # dictionnaire indexé par adresse Modbus.
        if len(active_addresses) == 1:
            return results[active_addresses[0]]

        return results

    # ------------------------------------------------------------------
    # Premier appel pour initialiser le coordinator
    # ------------------------------------------------------------------

    initial_data = await async_update_data()

    # battery_address ne sert qu'au libellé des entités, résolu par adresse.
    if multi_pack:
        initial_addresses = {
            addr: (data[0] if data and data[0] else f"0x{addr:02X}")
            for addr, data in (initial_data or {}).items()
        }
    else:
        battery_address = initial_data[0] if initial_data else None
        initial_addresses = {addresses[0]: battery_address}

    coordinator = DataUpdateCoordinator(
        hass,
        _LOGGER,
        name=f"seplos_bms_sensor_{config_battery_address}",
        update_method=async_update_data,
        update_interval=timedelta(seconds=poll_interval),
    )

    _LOGGER.debug("async_refresh data generate_sensors called (addr=%s)", config_battery_address)
    await coordinator.async_refresh()

    # ------------------------------------------------------------------
    # Définition des capteurs PIA (pack global)
    # ------------------------------------------------------------------

    # Un jeu de capteurs par adresse configurée.
    sensors = []
    for _mb in addresses:
        battery_address = initial_addresses.get(_mb)
        pia_sensors = [
            SeplosBMSSensorBase(
                coordinator, connector_info, "pack_voltage",
                "Pack Voltage", "V", "mdi:flash-circle",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "current",
                "Current", "A", "mdi:current-ac",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "remaining_capacity",
                "Remaining Capacity", "Ah", "mdi:battery-charging-wireless",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "total_capacity",
                "Total Capacity", "Ah", "mdi:battery",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "total_discharge_capacity",
                "Total Discharge Capacity", "Ah", "mdi:battery-discharging",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "soc",
                "State of Charge", "%", "mdi:gauge",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "soh",
                "State of Health", "%", "mdi:gauge",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "cycle",
                "Cycle", None, "mdi:numeric",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "avg_cell_voltage",
                "Avg Cell Voltage", "V", "mdi:battery-20",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "avg_cell_temperature",
                "Avg Cell Temperature", "°C", "mdi:thermometer",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "max_cell_voltage",
                "Max Cell Voltage", "V", "mdi:battery-high",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "min_cell_voltage",
                "Min Cell Voltage", "V", "mdi:battery-low",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "max_cell_temperature",
                "Max Cell Temperature", "°C", "mdi:thermometer-chevron-up",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "min_cell_temperature",
                "Min Cell Temperature", "°C", "mdi:thermometer-chevron-down",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "max_discharge_current",
                "Max Discharge Current", "A", "mdi:current-dc",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "max_charge_current",
                "Max Charge Current", "A", "mdi:current-dc",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
        ]

        # ------------------------------------------------------------------
        # Définition des capteurs PIB (cellules individuelles)
        # ------------------------------------------------------------------

        pib_sensors = [
            SeplosBMSSensorBase(
                coordinator, connector_info, f"cell{i}_voltage",
                f"Cell {i} Voltage", "V", "mdi:battery",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            )
            for i in range(1, 17)
        ] + [
            SeplosBMSSensorBase(
                coordinator, connector_info, "cell_temperature_1",
                "Cell Temperature 1", "°C", "mdi:thermometer",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "cell_temperature_2",
                "Cell Temperature 2", "°C", "mdi:thermometer",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "cell_temperature_3",
                "Cell Temperature 3", "°C", "mdi:thermometer",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "cell_temperature_4",
                "Cell Temperature 4", "°C", "mdi:thermometer",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "environment_temperature",
                "Environment Temperature", "°C", "mdi:thermometer-lines",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
            SeplosBMSSensorBase(
                coordinator, connector_info, "power_temperature",
                "Power Temperature", "°C", "mdi:thermometer-lines",
                battery_address=battery_address, sensor_prefix=sensor_prefix, entry_id=entry_id,
                modbus_address=_mb, multi_pack=multi_pack
            ),
        ]

        sensors += pia_sensors + pib_sensors

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
                 modbus_address=None, multi_pack=False):
        super().__init__(coordinator)
        # Entity name already includes prefix; prevent HA 2024+ from doubling.
        self._attr_has_entity_name = False
        self._port = port
        self._attribute = attribute
        self._name = name
        self._unit = unit
        self._icon = icon
        self._battery_address = battery_address
        self._sensor_prefix = sensor_prefix
        self._set_sensor_attributes(attribute)
        self._entry_id = entry_id
        # Modbus address this sensor reads from; indexes the coordinator
        # payload in multi-pack mode.
        self._modbus_address = modbus_address
        self._multi_pack = multi_pack

        # Device info for V3 BMS. One device per pack when the entry covers
        # several addresses, otherwise a single device for the entry.
        if multi_pack and modbus_address is not None:
            self._attr_device_info = DeviceInfo(
                identifiers={("bms_connector", f"seplos_v3_{entry_id}_{modbus_address}")},
                name=f"{sensor_prefix} 0x{modbus_address:02X}",
                manufacturer="Seplos",
                model="V3 BMS",
                sw_version="Unknown",
            )
        else:
            self._attr_device_info = DeviceInfo(
                identifiers={("bms_connector", f"seplos_v3_{entry_id}")},
                name=f"{sensor_prefix}",
                manufacturer="Seplos",
                model="V3 BMS",
                sw_version="Unknown",
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
        # A pack silent at startup has no parsed address string; fall back to
        # its configured Modbus address.
        address = self._battery_address
        if address is None and self._modbus_address is not None:
            address = f"0x{self._modbus_address:02X}"
        prefix = f"{self._sensor_prefix} - {address} -"
        return f"{prefix} {self._name}"

    @property
    def unique_id(self):
        # The Modbus address is only part of the id when an entry covers
        # several packs.
        if self._multi_pack and self._modbus_address is not None:
            return f"bms_connector_v3_{self._entry_id}_{self._modbus_address}_{self._name}"
        return f"bms_connector_v3_{self._entry_id}_{self._name}"

    def _pack_data(self):
        """Return this sensor's 5-tuple of parsed tables, or None.

        Multi-pack coordinators return {modbus_address: tuple}; single-pack
        ones return the tuple directly.
        """
        data = self.coordinator.data
        if isinstance(data, dict):
            return data.get(self._modbus_address)
        return data

    @property
    def state(self):
        if not self._attribute:
            return super().state

        value = _MISSING
        pack_data = self._pack_data()

        if isinstance(pack_data, tuple):
            battery_address_data, pia_data, pib_data, system_details_data, protection_settings_data = \
                pack_data
            # Cherche dans PIA, puis PIB, puis les autres objets
            # Utilise _MISSING comme sentinel pour distinguer "absent" de "valeur 0"
            for data_obj in (pia_data, pib_data, system_details_data, protection_settings_data):
                result = self.get_value(data_obj)
                if result is not _MISSING:
                    value = result
                    break
        elif pack_data is not None:
            value = self.get_value(pack_data)

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
