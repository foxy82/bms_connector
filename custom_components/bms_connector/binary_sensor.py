"""Binary sensors for BMS Connector (V3 PIC alarms and SFA switch states)."""

import logging

from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .bms.seplos.v3.sensors import pack_data, seplos_v3_device_info
from .bms.seplos.v3.vendor_spec import PIC_BITS, SFA_BITS
from .const import DOMAIN

_LOGGER = logging.getLogger(__name__)

# Index of the PIC and SFA tables in a pack's data tuple.
_PIC, _SFA = 4, 5

# Readable flags decoded from PIC: (name, V3PIC attribute, device class, icon,
# diagnostic, attribute listing the active items as an extra state attribute).
STATUS_SENSORS = (
    ("Heating", "heating", None, "mdi:heating-coil", False, None),
    ("Charging", "charging", BinarySensorDeviceClass.BATTERY_CHARGING, None, False, None),
    ("Discharging", "discharging", None, "mdi:battery-arrow-down", False, None),
    ("Alarm", "active_alarms", BinarySensorDeviceClass.PROBLEM, None, False, "alarms"),
    ("Fault", "active_faults", BinarySensorDeviceClass.PROBLEM, None, False, "faults"),
    ("Charge FET", "charge_fet", None, "mdi:electric-switch", True, None),
    ("Discharge FET", "discharge_fet", None, "mdi:electric-switch", True, None),
    ("Current Limiting", "current_limiting", None, "mdi:speedometer-slow", True, None),
)


class SeplosV3BinarySensor(CoordinatorEntity, BinarySensorEntity):
    """Read-only alarm bit (PIC) or switch state bit (SFA).

    One entity per protocol bit, so they start disabled; the readable status
    sensors cover the common ones.
    """

    _attr_entity_registry_enabled_default = False

    def __init__(self, coordinator, kind, index, name, modbus_address,
                 sensor_prefix, entry_id, multi_pack):
        super().__init__(coordinator)
        self._kind = kind
        self._index = index
        self._modbus_address = modbus_address
        battery_address = f"0x{modbus_address:02X}"
        self._attr_has_entity_name = False
        self._attr_name = f"{sensor_prefix} {battery_address} {name}"
        # Index is part of the identity: PIC/SFA both contain duplicate
        # "Reservation" labels.
        self._attr_unique_id = (
            f"bms_connector_v3_{entry_id}_{kind}_{battery_address}_{index}_{name}"
        )
        self._attr_device_info = seplos_v3_device_info(
            entry_id, sensor_prefix, modbus_address, multi_pack,
            settings=kind == "sfa",
        )

    @property
    def is_on(self):
        data = pack_data(self.coordinator, self._modbus_address)
        table = data[_PIC if self._kind == "pic" else _SFA] if data else None
        if table is None:
            return False
        try:
            return bool(table.bits[self._index])
        except (IndexError, TypeError):
            return False


class SeplosV3StatusBinarySensor(CoordinatorEntity, BinarySensorEntity):
    """One readable flag decoded from a pack's PIC status table.

    A list-valued flag (alarms, faults) is on while the list is non-empty and
    exposes the list as an extra state attribute.
    """

    # Entity name already carries the prefix; prevent HA 2024+ doubling.
    _attr_has_entity_name = False

    def __init__(self, coordinator, description, entry_id, sensor_prefix,
                 modbus_address, multi_pack):
        super().__init__(coordinator)
        name, self._flag, device_class, icon, diagnostic, self._list_attr = description
        self._modbus_address = modbus_address
        self._attr_device_class = device_class
        if icon:
            self._attr_icon = icon
        if diagnostic:
            self._attr_entity_category = EntityCategory.DIAGNOSTIC
        self._attr_name = f"{sensor_prefix} - 0x{modbus_address:02X} - {name}"
        if multi_pack:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_{modbus_address}_{name}"
        else:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_{name}"
        self._attr_device_info = seplos_v3_device_info(
            entry_id, sensor_prefix, modbus_address, multi_pack
        )

    def _pic(self):
        data = pack_data(self.coordinator, self._modbus_address)
        return data[_PIC] if data else None

    @property
    def available(self) -> bool:
        return super().available and self._pic() is not None

    @property
    def is_on(self):
        pic = self._pic()
        return bool(getattr(pic, self._flag)) if pic is not None else None

    @property
    def extra_state_attributes(self):
        pic = self._pic()
        if self._list_attr is None or pic is None:
            return None
        return {self._list_attr: getattr(pic, self._flag)}


def build_entities(context, entry_id):
    """Binary sensors for every pack of an entry that reads status flags."""
    if not context["read_status_flags"]:
        return []
    coordinator = context["coordinator"]
    prefix = context["sensor_prefix"]
    multi_pack = context["multi_pack"]
    entities = []
    for addr in context["addresses"]:
        entities += [
            SeplosV3StatusBinarySensor(coordinator, description, entry_id, prefix,
                                       addr, multi_pack)
            for description in STATUS_SENSORS
        ]
        for kind, spec in (("pic", PIC_BITS), ("sfa", SFA_BITS)):
            entities += [
                SeplosV3BinarySensor(coordinator, kind, index, item["name"], addr,
                                     prefix, entry_id, multi_pack)
                for index, item in enumerate(spec)
            ]
    return entities


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry,
                            async_add_entities: AddEntitiesCallback):
    if entry.data.get("bms_type") != "SEPLV3":
        return
    # The sensor platform creates the coordinator. Register here first, then
    # either consume it now (if ready) or let the sensor platform call back.
    pending = hass.data.setdefault(DOMAIN, {}).setdefault("binary_add", {})
    pending[entry.entry_id] = async_add_entities

    context = hass.data.get(DOMAIN, {}).get("coordinators", {}).get(entry.entry_id)
    if context is None:
        _LOGGER.debug("Waiting for V3 coordinator for %s", entry.entry_id)
        return

    pending.pop(entry.entry_id, None)
    async_add_entities(build_entities(context, entry.entry_id), True)
